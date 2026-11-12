"""Builds worlds/drone_handoff_realmap.wbt from real, free geographic data.

Ground texture : Esri World Imagery satellite tiles (z18, Web Mercator), with
                 OpenStreetMap standard tiles as automatic fallback.
Elevation      : SRTM via Mapzen/AWS public terrain tiles (terrarium encoding, z15)
3D buildings   : OSM building footprints via the free Overpass API (no key),
                 seated on the terrain. Only spawn pads and terminal hover
                 zones are kept clear; corridor buildings stay as obstacles
                 and are exported to textures/buildings_obstacles.json for
                 the controllers' avoidance layer.

The mission geometry is unchanged (world origin = real anchor point); only the
ground, spawn heights, background, buildings and viewpoint are replaced.
Usage: python build_realmap_world.py
"""
import io
import json
import math
import os
import time
import urllib.parse
import urllib.request

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(ROOT, "worlds")
TEXDIR = os.path.join(WORLDS, "textures")
SRC_WORLD = os.path.join(WORLDS, "drone_handoff.wbt")
DST_WORLD = os.path.join(WORLDS, "drone_handoff_realmap.wbt")

# Washington Monument, National Mall, Washington DC — flat, iconic, free data
ANCHOR_LAT = 38.8895
ANCHOR_LON = -77.0353
LOCATION = "Washington Monument, National Mall, Washington DC"
BOX_M = 400.0            # square world extent, centered on the anchor
GRID_N = 101             # ElevationGrid points per side
GRID_SPACING = BOX_M / (GRID_N - 1)

OSM_UA = "webots-drone-handoff-sim/1.0 (local simulation)"
MAX_MISSION_RELIEF_M = 3.5   # relief in the flight area; above this, use a flat floor

# Esri World Imagery puts {y} before {x} in the REST path; OSM uses {x}/{y}.
# Both share the identical Web Mercator {z}/{x}/{y} slippy-tile grid.
ESRI_TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                 "World_Imagery/MapServer/tile/{z}/{ty}/{tx}")
OSM_TILE_URL = "https://tile.openstreetmap.org/{z}/{tx}/{ty}.png"
ESRI_ATTRIBUTION = ("Esri, Maxar, Earthstar Geographics, and the GIS User Community")
OVERPASS_URLS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
)

TARGET_XY = (6.0, 4.0)
TARGET_END_XY = (16.5, 11.0)  # Phase 14: moving-target lane end (6,4)->(16.5,11) at [0.09, 0.06] m/s
HOME_SCOUT = (-6.0, -4.0)
HOME_F1 = (-4.0, -3.0)
HOME_F2 = (-4.0, 3.0)
LOITER_XY = (15.0, 10.0)    # Phase 14: clear of the moving-target corridor
SPAWN_CLEAR_M = 4.0         # keep drone takeoff pads clear
HOVER_CLEAR_M = 5.0         # keep target/loiter hover zones clear
CORRIDOR_NEAR_M = 2.5       # "blocks the path" = bbox within this distance of a flight segment
# Phase 9: deterministic staggered corridor obstacles (multi-building slalom).
# Injected whenever fewer than 2 real OSM buildings block the follower paths so
# the multi-obstacle avoidance and wind-rejection gates are always exercised.
CORRIDOR_BLOCKS = (
    ("osm_building_corridor_block_1", (-1.2, -1.0), (2.6, 2.6, 9.0)),
    ("osm_building_corridor_block_2", (2.2, 2.2), (2.6, 2.6, 9.0)),
)

BG_REPLACEMENT = '''Background {
  skyColor [
    0.529 0.808 0.98
  ]
}
'''

ELEV_GRID_BLOCK = '''Solid {
  name "real_terrain"
  translation -200 -200 0
  children [
    Shape {
      appearance PBRAppearance {
        baseColorMap ImageTexture {
          url [ "textures/realmap_esri.png" ]
        }
        roughness 1
        metalness 0
      }
      geometry ElevationGrid {
        height [
$HEIGHTS$
        ]
        xDimension 101
        xSpacing 4
        yDimension 101
        ySpacing 4
        thickness 10
      }
      castShadows FALSE
    }
  ]
  boundingObject ElevationGrid {
    height [
$HEIGHTS$
    ]
    xDimension 101
    xSpacing 4
    yDimension 101
    ySpacing 4
    thickness 10
  }
  locked TRUE
}
'''

FLAT_FLOOR_BLOCK = '''Floor {
  size 400 400
  tileSize 400 400
  appearance PBRAppearance {
    baseColorMap ImageTexture {
      url [ "textures/realmap_esri.png" ]
    }
    roughness 1
    metalness 0
  }
}
'''


def http_get(url, binary=True, ua=OSM_UA, attempts=3, backoff=1.5):
    for attempt in range(attempts):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": ua})
            with urllib.request.urlopen(req, timeout=45) as r:
                data = r.read()
            return data if binary else data.decode("utf-8")
        except Exception as e:
            if attempt == attempts - 1:
                raise RuntimeError(f"download failed: {url}: {e}")
            time.sleep(backoff)


def tile_xy_float(z, lat, lon):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def corner_latlon():
    half = BOX_M / 2.0
    dlat = half / 111320.0
    dlon = half / (111320.0 * math.cos(math.radians(ANCHOR_LAT)))
    return {"north": ANCHOR_LAT + dlat, "south": ANCHOR_LAT - dlat,
            "west": ANCHOR_LON - dlon, "east": ANCHOR_LON + dlon}


def fetch_ground_texture():
    """Satellite imagery (Esri) with OSM-vector fallback. Returns (path, source)."""
    path = os.path.join(TEXDIR, "realmap_esri.png")
    if os.path.exists(path):
        print("  using cached realmap_esri.png (Esri satellite)")
        return path, "esri"
    z = 18
    c = corner_latlon()
    x0f, y0f = tile_xy_float(z, c["north"], c["west"])
    x1f, y1f = tile_xy_float(z, c["south"], c["east"])
    tx0, ty0 = math.floor(x0f), math.floor(y0f)
    tx1, ty1 = math.floor(x1f), math.floor(y1f)

    success = False
    for source, url_tpl in (("esri", ESRI_TILE_URL), ("osm", OSM_TILE_URL)):
        canvas = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256), (255, 255, 255))
        try:
            for ty in range(ty0, ty1 + 1):
                for tx in range(tx0, tx1 + 1):
                    url = url_tpl.format(z=z, tx=tx, ty=ty)
                    img = Image.open(io.BytesIO(http_get(url)))
                    canvas.paste(img.convert("RGB"), ((tx - tx0) * 256, (ty - ty0) * 256))
                    print(f"  {source} tile {tx},{ty} ok")
            success = True
            break
        except RuntimeError as e:
            print(f"  {source} tiles failed ({e}) -> falling back to next source")
    if not success:
        raise RuntimeError("all ground imagery sources failed")

    px0 = x0f * 256 - tx0 * 256
    py0 = y0f * 256 - ty0 * 256
    px1 = x1f * 256 - tx0 * 256
    py1 = y1f * 256 - ty0 * 256
    crop = canvas.crop((int(round(px0)), int(round(py0)), int(round(px1)), int(round(py1))))
    img = crop.resize((1024, 1024), Image.LANCZOS)
    os.makedirs(TEXDIR, exist_ok=True)
    img.save(path)
    return path, source


def fetch_elevation_grid():
    """Returns E[j][i] (m) on the GRID_N x GRID_N grid, j=north index."""
    cache = os.path.join(TEXDIR, "elevation_cache.npy")
    if os.path.exists(cache):
        print("  using cached elevation")
        return np.load(cache)
    z = 15
    c = corner_latlon()
    x0f, y0f = tile_xy_float(z, c["north"], c["west"])
    x1f, y1f = tile_xy_float(z, c["south"], c["east"])
    tx0, ty0 = math.floor(x0f), math.floor(y0f)
    tx1, ty1 = math.floor(x1f), math.floor(y1f)

    canvas_w = (tx1 - tx0 + 1) * 256
    canvas_h = (ty1 - ty0 + 1) * 256
    elev = np.zeros((canvas_h, canvas_w), dtype=np.float64)
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{tx}/{ty}.png"
            data = http_get(url)
            img = np.asarray(Image.open(io.BytesIO(data)).convert("RGB")).astype(np.float64)
            tile = (img[:, :, 0] * 256.0 + img[:, :, 1] + img[:, :, 2] / 256.0) - 32768.0
            y_off = (ty - ty0) * 256
            x_off = (tx - tx0) * 256
            elev[y_off:y_off + 256, x_off:x_off + 256] = tile
            print(f"  elev tile {tx},{ty} ok")

    half = BOX_M / 2.0
    E = np.zeros((GRID_N, GRID_N), dtype=np.float64)
    for j in range(GRID_N):
        north = ANCHOR_LAT + (half - j * GRID_SPACING) / 111320.0
        for i in range(GRID_N):
            east = ANCHOR_LON + (-half + i * GRID_SPACING) / (111320.0 * math.cos(math.radians(ANCHOR_LAT)))
            xf, yf = tile_xy_float(z, north, east)
            px = xf * 256 - tx0 * 256
            py = yf * 256 - ty0 * 256
            x0, y0 = int(px), int(py)
            fx, fy = px - x0, py - y0
            v = (elev[y0, x0] * (1 - fx) * (1 - fy) + elev[y0, x0 + 1] * fx * (1 - fy)
                 + elev[y0 + 1, x0] * (1 - fx) * fy + elev[y0 + 1, x0 + 1] * fx * fy)
            E[j, i] = v
    os.makedirs(TEXDIR, exist_ok=True)
    np.save(cache, E)
    return E


def terrain_at(E, wx, wy):
    half = BOX_M / 2.0
    fx = (wx + half) / GRID_SPACING
    fy = (half - wy) / GRID_SPACING
    x0, y0 = int(math.floor(fx)), int(math.floor(fy))
    x1, y1 = min(x0 + 1, GRID_N - 1), min(y0 + 1, GRID_N - 1)
    ax, ay = fx - x0, fy - y0
    return (E[y0, x0] * (1 - ax) * (1 - ay) + E[y0, x1] * ax * (1 - ay)
            + E[y1, x0] * (1 - ax) * ay + E[y1, x1] * ax * ay)


def _bbox_dist_to_point(minx, miny, maxx, maxy, px, py):
    cx = min(maxx, max(minx, px))
    cy = min(maxy, max(miny, py))
    return math.hypot(px - cx, py - cy)


def _clear_of_pads_and_target(minx, miny, maxx, maxy):
    """Only skip buildings that physically overlap a spawn pad, a hover spot, or the
    moving-target lane. Buildings in the flight corridor are KEPT so drones must
    route around them."""
    for px, py in (HOME_SCOUT, HOME_F1, HOME_F2):
        if _bbox_dist_to_point(minx, miny, maxx, maxy, px, py) < SPAWN_CLEAR_M:
            return False
    for px, py in (TARGET_XY, LOITER_XY):
        if _bbox_dist_to_point(minx, miny, maxx, maxy, px, py) < HOVER_CLEAR_M:
            return False
    # Phase 14: keep the moving-target trajectory lane (TARGET_XY -> TARGET_END_XY) clear
    if _bbox_near_segment(minx, miny, maxx, maxy, TARGET_XY, TARGET_END_XY, HOVER_CLEAR_M):
        return False
    return True


def _bbox_near_segment(minx, miny, maxx, maxy, a, b, near_m):
    """True if the bbox comes within near_m of segment a->b (sampled every 1 m)."""
    steps = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1])))
    for i in range(steps + 1):
        t = i / steps
        px = a[0] + (b[0] - a[0]) * t
        py = a[1] + (b[1] - a[1]) * t
        if _bbox_dist_to_point(minx, miny, maxx, maxy, px, py) < near_m:
            return True
    return False


def fetch_osm_buildings_wbt(terrain_fn):
    """Fetches OSM building footprints via Overpass and returns
    (wbt_text, placed, skipped, corridor_blocks_injected).

    Only spawn pads (4 m) and terminal hover zones (5 m) are kept clear; buildings
    across the flight corridors are kept as obstacles. All placed buildings are
    exported to textures/buildings_obstacles.json for the avoidance layer. If no
    real building blocks a follower flight path, one deterministic corridor
    obstacle is injected so avoidance is always exercised.
    """
    dlat = (BOX_M / 2.0) / 111320.0
    dlon = (BOX_M / 2.0) / (111320.0 * math.cos(math.radians(ANCHOR_LAT)))
    south, north = ANCHOR_LAT - dlat, ANCHOR_LAT + dlat
    west, east = ANCHOR_LON - dlon, ANCHOR_LON + dlon
    query = f"""
    [out:json][timeout:25];
    (
      way["building"]({south},{west},{north},{east});
    );
    out body;
    >;
    out skel qt;
    """
    data = None
    for endpoint in OVERPASS_URLS:
        url = endpoint + "?data=" + urllib.parse.quote(query)
        try:
            data = json.loads(http_get(url, binary=False, ua="WebotsDroneSim/1.0 (local simulation)",
                                       attempts=4, backoff=4.0))
            break
        except RuntimeError as e:
            print(f"  Overpass {endpoint} failed: {e}")
    if data is None:
        raise RuntimeError("all Overpass endpoints failed")
    nodes = {el["id"]: (el["lat"], el["lon"]) for el in data["elements"] if el["type"] == "node"}
    m_per_deg_lon = 111320.0 * math.cos(math.radians(ANCHOR_LAT))
    half = BOX_M / 2.0

    def emit_solid(name, cx, cy, sx, sy, height):
        # footprints crossing the box edge have centers outside the terrain grid:
        # clamp the sample point into the box for the ground height
        sx_p = max(-half + 1.0, min(half - 1.0, cx))
        sy_p = max(-half + 1.0, min(half - 1.0, cy))
        z_ground = terrain_fn(sx_p, sy_p)
        cz = z_ground + height / 2.0
        return f'''
Solid {{
  name "{name}"
  translation {cx:.2f} {cy:.2f} {cz:.2f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.78 0.76 0.72
        roughness 0.9
        metalness 0.1
      }}
      geometry Box {{ size {sx:.2f} {sy:.2f} {height:.2f} }}
    }}
  ]
  boundingObject Box {{ size {sx:.2f} {sy:.2f} {height:.2f} }}
  locked TRUE
}}'''

    blocks, placed, skipped = [], 0, 0
    obstacles_list = []
    for el in data["elements"]:
        if el["type"] != "way" or "nodes" not in el:
            continue
        pts = []
        for nid in el["nodes"]:
            if nid in nodes:
                lat, lon = nodes[nid]
                pts.append(((lon - ANCHOR_LON) * m_per_deg_lon,
                            (lat - ANCHOR_LAT) * 111320.0))
        if len(pts) < 4:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
        sx, sy = max(2.0, maxx - minx), max(2.0, maxy - miny)
        if not _clear_of_pads_and_target(minx, miny, maxx, maxy):
            skipped += 1
            continue
        tags = el.get("tags", {})
        levels = float(tags.get("building:levels", 2))
        height = float(tags.get("height", levels * 3.5))
        blocks.append(emit_solid(f"osm_building_{el['id']}", cx, cy, sx, sy, height))
        obstacles_list.append({
            "id": el["id"],
            "minx": round(minx, 2), "miny": round(miny, 2),
            "maxx": round(maxx, 2), "maxy": round(maxy, 2),
            "cx": round(cx, 2), "cy": round(cy, 2),
            "height": round(height, 2),
        })
        placed += 1

    # Phase 9: guarantee a multi-obstacle slalom — count the real OSM buildings
    # that physically block a follower path; when fewer than 2, inject the two
    # deterministic staggered corridor blocks.
    segs = [(HOME_F1, TARGET_XY), (HOME_F2, TARGET_XY)]
    natural_blocks = sum(
        1 for obs in obstacles_list
        if any(_bbox_near_segment(obs["minx"], obs["miny"], obs["maxx"], obs["maxy"],
                                  sa, sb, CORRIDOR_NEAR_M) for (sa, sb) in segs))
    corridor_blocks_injected = 0
    if natural_blocks < 2:
        for name, (bx, by), (bsx, bsy, bh) in CORRIDOR_BLOCKS:
            blocks.append(emit_solid(name, bx, by, bsx, bsy, bh))
            obstacles_list.append({
                "id": name.replace("osm_building_", ""),
                "minx": round(bx - bsx / 2, 2), "miny": round(by - bsy / 2, 2),
                "maxx": round(bx + bsx / 2, 2), "maxy": round(by + bsy / 2, 2),
                "cx": bx, "cy": by, "height": bh,
            })
            placed += 1
            corridor_blocks_injected += 1
            print(f"  injected deterministic corridor obstacle '{name}' at ({bx}, {by}) "
                  f"({bsx}x{bsy}x{bh} m) — only {natural_blocks} natural OSM building(s) "
                  f"block the follower paths (multi-obstacle slalom)")

    os.makedirs(TEXDIR, exist_ok=True)
    with open(os.path.join(TEXDIR, "buildings_obstacles.json"), "w") as f:
        json.dump(obstacles_list, f, indent=2)
    return "\n".join(blocks), placed, skipped, corridor_blocks_injected


def build_world_text(use_elevation, E, heights_list, meta, buildings_text=""):
    with open(SRC_WORLD, encoding="utf-8") as f:
        src = f.read()

    src = src.replace(
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"\n', "")
    src = src.replace(
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"\n', "")
    if use_elevation:
        src = src.replace(
            'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto"\n', "")

    src = src.replace(
        '''WorldInfo {
  info [
    "Cooperative Drone Target Handoff and Visual Servoing Simulation"
  ]
''',
        f'''WorldInfo {{
  info [
    "Cooperative Drone Target Handoff and Visual Servoing Simulation"
    "Real geography: {LOCATION} (anchor {ANCHOR_LAT} N, {ANCHOR_LON} W = world origin)"
    "Ground texture: Esri World Imagery satellite tiles ({ESRI_ATTRIBUTION})"
    "3D buildings: OpenStreetMap footprints via Overpass API, (c) OpenStreetMap contributors (ODbL)"
    "Elevation: SRTM via Mapzen/AWS public terrain tiles (terrarium z15)"
  ]
''')

    src = src.replace(
        '''Viewpoint {
  orientation -0.28 0.25 0.92 1.65
  position -6.5 -14.0 11.5
  follow "Scout_Drone"
}''',
        '''Viewpoint {
  orientation 1 0 0 -1.5708
  position 0 0 300
}
''' + (buildings_text + "\n" if buildings_text else ""))

    src = src.replace(
        '''TexturedBackground {
}

TexturedBackgroundLight {
}

Floor {
  size 100 100
  tileSize 2 2
}''',
        BG_REPLACEMENT + (ELEV_GRID_BLOCK if use_elevation else FLAT_FLOOR_BLOCK).replace("$HEIGHTS$", heights_list))

    if use_elevation:
        z_t = terrain_at(E, TARGET_XY[0], TARGET_XY[1])
        z_s = terrain_at(E, HOME_SCOUT[0], HOME_SCOUT[1])
        z_f1 = terrain_at(E, HOME_F1[0], HOME_F1[1])
        z_f2 = terrain_at(E, HOME_F2[0], HOME_F2[1])
    else:
        z_t = z_s = z_f1 = z_f2 = 0.0
    meta["terrain_mode"] = "elevation" if use_elevation else "flat"
    meta["terrain_at_target_m"] = round(z_t, 3)

    src = src.replace("translation 6.0 4.0 0.05", f"translation 6.0 4.0 {z_t + 0.05:.2f}")
    src = src.replace("translation -6 -4 0.1", f"translation -6 -4 {z_s + 0.1:.2f}")
    src = src.replace("translation -4.0 -3.0 0.1", f"translation -4.0 -3.0 {z_f1 + 0.1:.2f}")
    src = src.replace("translation -4.0 3.0 0.1", f"translation -4.0 3.0 {z_f2 + 0.1:.2f}")
    return src


def main():
    print("== Building real-geography world (Esri satellite + OSM 3D buildings) ==")
    print(f"Location : {LOCATION}")
    print(f"Anchor   : {ANCHOR_LAT} N, {ANCHOR_LON} W  |  box {BOX_M:.0f} x {BOX_M:.0f} m")
    print("Downloading ground imagery tiles (z18, Esri first)...")
    _, imagery_source = fetch_ground_texture()
    print("Downloading SRTM elevation tiles (z15, terrarium)...")
    E = fetch_elevation_grid()

    t_target = terrain_at(E, TARGET_XY[0], TARGET_XY[1])
    E_centered = E - t_target
    mission_vals = [terrain_at(E_centered, x, y)
                    for x in np.arange(-8.0, 11.0, 1.0) for y in np.arange(-6.0, 9.0, 1.0)]
    mission_relief = max(mission_vals) - min(mission_vals)
    print(f"Terrain at target: {t_target:.2f} m | mission-box relief: {mission_relief:.2f} m")

    use_elevation = mission_relief <= MAX_MISSION_RELIEF_M
    if use_elevation:
        print("Mission area is flat enough -> keeping real SRTM elevation (centered at target).")
        heights_list = ",\n          ".join(f"{v:.3f}" for v in E_centered.flatten(order="C"))
    else:
        print(f"Mission-box relief exceeds {MAX_MISSION_RELIEF_M} m -> flat floor with the real map texture.")
        heights_list = ""

    terrain_fn = (lambda wx, wy: terrain_at(E_centered, wx, wy)) if use_elevation else (lambda wx, wy: 0.0)
    buildings_text, placed, skipped, corridor_injected = "", 0, 0, 0
    print("Downloading OSM 3D building footprints (Overpass API)...")
    try:
        buildings_text, placed, skipped, corridor_injected = fetch_osm_buildings_wbt(terrain_fn)
        print(f"  3D buildings: {placed} placed, {skipped} skipped (spawn pads / hover zones)")
    except Exception as e:
        print(f"  3D buildings skipped (Overpass unavailable): {e}")

    meta = {
        "location": LOCATION,
        "anchor": [ANCHOR_LAT, ANCHOR_LON],
        "box_m": BOX_M,
        "image_px": 1024,
        "imagery_source": imagery_source,
        "buildings_placed": placed,
        "buildings_skipped_pads_hover": skipped,
        "corridor_block_injected": bool(corridor_injected),
        "corridor_blocks_injected": corridor_injected,
        "mission_relief_m": round(mission_relief, 2),
        "attribution": (f"Imagery: {ESRI_ATTRIBUTION} | Buildings (c) OpenStreetMap "
                        "contributors (ODbL) | Elevation SRTM (Mapzen/AWS terrain tiles)"),
    }
    world_text = build_world_text(use_elevation, E_centered, heights_list, meta, buildings_text)
    with open(DST_WORLD, "w", encoding="utf-8") as f:
        f.write(world_text)
    with open(os.path.join(TEXDIR, "realmap_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    old_cache = os.path.join(TEXDIR, "realmap.png")
    if os.path.exists(old_cache):
        os.remove(old_cache)
        print(f"Removed obsolete cached vector map {old_cache}")

    print(f"\nWrote {DST_WORLD}")
    print(f"Terrain mode : {meta['terrain_mode']}  (mission-box relief {meta['mission_relief_m']} m)")
    print(f"Imagery      : {imagery_source.upper()}  |  buildings placed: {placed}")
    print("Done.")


if __name__ == "__main__":
    main()
