# Key Results Evidence (JSON snapshots)

## results_mc/mc_summary.json
```json
{
  "n_trials": 3,
  "all_passed": false,
  "pass_count": 1,
  "milestone_pass_rates": {
    "1_raycast_precision": 3,
    "2_reception_visual_id": 3,
    "3_airspace_deconfliction": 3,
    "4_drift_correction_visual_homing": 1,
    "5_obstacle_avoidance": 3,
    "6_live_gps_wander_ibvs": 1,
    "7_mapless_sensor_avoidance": 3,
    "8_degraded_radio_retransmission": 2,
    "9_multi_obstacle_and_wind_rejection": 1,
    "10_fiducial_geometry_verification": 2
  },
  "terminal_error_m": {
    "min": 0.028243557460914483,
    "max": 0.07063375219293191,
    "mean": 0.040997557158585635
  },
  "trials": [
    {
      "seed": 0,
      "mission_success": true,
      "webots_exit_code": 0,
      "milestones": {
        "1_raycast_precision": true,
        "2_reception_visual_id": true,
        "3_airspace_deconfliction": true,
        "4_drift_correction_visual_homing": true,
        "5_obstacle_avoidance": true,
        "6_live_gps_wander_ibvs": true,
        "7_mapless_sensor_avoidance": true,
        "8_degraded_radio_retransmission": true,
        "9_multi_obstacle_and_wind_rejection": true,
        "10_fiducial_geometry_verification": true
      },
      "terminal_errors_m": {
        "Follower_Drone_01": 0.03180569621443358,
        "Follower_Drone_02": 0.03330722276606256
      },
      "sim_time_s": 165.52
    },
    {
      "seed": 1,
      "mission_success": false,
      "webots_exit_code": 1,
      "milestones": {
        "1_raycast_precision": true,
        "2_reception_visual_id": true,
        "3_airspace_deconfliction": true,
        "4_drift_correction_visual_homing": false,
        "5_obstacle_avoidance": true,
        "6_live_gps_wander_ibvs": false,
        "7_mapless_sensor_avoidance": true,
        "8_degraded_radio_retransmission": false,
        "9_multi_obstacle_and_wind_rejection": false,
        "10_fiducial_geometry_verification": false
      },
      "terminal_errors_m": {
        "Follower_Drone_01": null,
        "Follower_Drone_02": null
      },
      "sim_time_s": 148.488
    },
    {
      "seed": 2,
      "mission_success": false,
      "webots_exit_code": 0,
      "milestones": {
        "1_raycast_precision": true,
        "2_reception_visual_id": true,
        "3_airspace_deconfliction": true,
        "4_drift_correction_visual_homing": false,
        "5_obstacle_avoidance": true,
        "6_live_gps_wander_ibvs": false,
        "7_mapless_sensor_avoidance": true,
        "8_degraded_radio_retransmission": true,
        "9_multi_obstacle_and_wind_rejection": false,
        "10_fiducial_geometry_verification": true
      },
      "terminal_errors_m": {
        "Follower_Drone_01": 0.07063375219293191,
        "Follower_Drone_02": 0.028243557460914483
      },
      "sim_time_s": 166.328
    }
  ]
}
```

## results/wind_log.csv (head)
```
t,fx_n,fy_n,fz_n,applied_scout,applied_f1,applied_f2
2.00,0.002,-0.011,0.004,0,0,0
4.00,0.014,-0.014,0.003,0,0,0
6.00,0.025,-0.003,-0.001,1,1,1
8.00,0.008,-0.018,-0.004,1,1,1
10.00,0.007,-0.004,-0.002,1,1,1
12.00,0.022,-0.012,0.003,1,1,1
14.00,0.016,-0.012,0.004,1,1,1
16.00,0.006,-0.004,-0.000,1,1,1
18.00,0.014,-0.018,-0.004,1,1,1
20.00,0.021,-0.003,-0.003,1,1,1
22.00,0.012,-0.013,0.002,1,1,1
24.00,0.007,-0.012,0.004,1,1,1
26.00,0.018,-0.003,0.001,1,1,1
28.00,0.021,-0.021,-0.003,1,1,1
30.00,0.006,0.003,-0.003,1,1,1
32.00,0.010,-0.021,0.001,1,1,1
34.00,0.025,-0.003,0.004,1,1,1
36.00,0.012,-0.011,0.002,1,1,1
38.00,0.003,-0.015,-0.003,1,1,1
40.00,0.023,-0.000,-0.004,1,1,1
42.00,0.021,-0.022,-0.000,1,1,1
44.00,0.001,0.001,0.004,1,1,1
46.00,0.015,-0.017,0.003,1,1,1
48.00,0.026,-0.008,-0.002,1,1,1
50.00,0.007,-0.007,-0.004,1,1,1
52.00,0.007,-0.016,-0.001,1,1,1
54.00,0.024,-0.003,0.003,1,1,1
56.00,0.016,-0.016,0.004,1,1,1
58.00,0.005,-0.008,-0.000,1,1,1
60.00,0.016,-0.007,-0.004,1,1,1
62.00,0.021,-0.017,-0.002,1,1,1
64.00,0.011,-0.001,0.002,1,1,1
66.00,0.008,-0.018,0.004,1,1,1
68.00,0.019,-0.005,0.001,1,1,1
70.00,0.019,-0.010,-0.003,1,1,1
72.00,0.006,-0.016,-0.003,1,1,1
74.00,0.011,0.001,0.001,1,1,1
76.00,0.025,-0.023,0.004,1,1,1
78.00,0.012,0.002,0.002,1,1,1
```
