use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_double, c_int, c_uchar, c_ushort, c_void};
use std::slice;

pub type WbDeviceTag = c_ushort;
pub type WbNodeRef = *mut c_void;
pub type WbFieldRef = *mut c_void;

extern "C" {
    fn wb_robot_init() -> c_int;
    fn wb_robot_cleanup();
    fn wb_robot_step(duration: c_int) -> c_int;
    fn wb_robot_get_time() -> c_double;
    fn wb_robot_get_basic_time_step() -> c_double;
    fn wb_robot_get_custom_data() -> *const c_char;
    fn wb_robot_get_device(name: *const c_char) -> WbDeviceTag;

    fn wb_motor_set_position(tag: WbDeviceTag, position: c_double);
    fn wb_motor_set_velocity(tag: WbDeviceTag, velocity: c_double);

    fn wb_gps_enable(tag: WbDeviceTag, sampling_period: c_int);
    fn wb_gps_get_values(tag: WbDeviceTag) -> *const c_double;
    fn wb_gps_get_speed_vector(tag: WbDeviceTag) -> *const c_double;

    fn wb_inertial_unit_enable(tag: WbDeviceTag, sampling_period: c_int);
    fn wb_inertial_unit_get_roll_pitch_yaw(tag: WbDeviceTag) -> *const c_double;

    fn wb_gyro_enable(tag: WbDeviceTag, sampling_period: c_int);
    fn wb_gyro_get_values(tag: WbDeviceTag) -> *const c_double;

    fn wb_camera_enable(tag: WbDeviceTag, sampling_period: c_int);
    fn wb_camera_get_image(tag: WbDeviceTag) -> *const c_uchar;
    fn wb_camera_get_width(tag: WbDeviceTag) -> c_int;
    fn wb_camera_get_height(tag: WbDeviceTag) -> c_int;

    fn wb_distance_sensor_enable(tag: WbDeviceTag, sampling_period: c_int);
    fn wb_distance_sensor_get_value(tag: WbDeviceTag) -> c_double;

    fn wb_emitter_send(tag: WbDeviceTag, data: *const c_void, size: c_int) -> c_int;

    fn wb_receiver_enable(tag: WbDeviceTag, sampling_period: c_int);
    fn wb_receiver_get_queue_length(tag: WbDeviceTag) -> c_int;
    fn wb_receiver_get_data(tag: WbDeviceTag) -> *const c_void;
    fn wb_receiver_get_data_size(tag: WbDeviceTag) -> c_int;
    fn wb_receiver_next_packet(tag: WbDeviceTag);

    fn wb_supervisor_node_get_from_def(name: *const c_char) -> WbNodeRef;
    fn wb_supervisor_node_get_position(node: WbNodeRef) -> *const c_double;
    fn wb_supervisor_node_add_force(node: WbNodeRef, force: *const c_double, relative: c_char);
    fn wb_supervisor_simulation_quit(status: c_int);
    fn wb_supervisor_node_get_root() -> WbNodeRef;
    fn wb_supervisor_node_get_field(node: WbNodeRef, name: *const c_char) -> WbFieldRef;
    fn wb_supervisor_field_get_count(field: WbFieldRef) -> c_int;
    fn wb_supervisor_field_get_mf_node(field: WbFieldRef, index: c_int) -> WbNodeRef;
    fn wb_supervisor_node_get_type_name(node: WbNodeRef) -> *const c_char;
    fn wb_supervisor_field_get_sf_string(field: WbFieldRef) -> *const c_char;
    fn wb_supervisor_field_get_sf_vec3f(field: WbFieldRef) -> *const c_double;
    fn wb_supervisor_field_set_sf_vec3f(field: WbFieldRef, values: *const c_double);
}

pub struct Robot;

impl Robot {
    pub fn new() -> Self {
        unsafe { wb_robot_init(); }
        Self
    }
    pub fn step(&self, ms: i32) -> bool {
        unsafe { wb_robot_step(ms as c_int) != -1 }
    }
    pub fn time(&self) -> f64 {
        unsafe { wb_robot_get_time() }
    }
    pub fn basic_time_step(&self) -> i32 {
        unsafe { wb_robot_get_basic_time_step() as i32 }
    }
    pub fn custom_data(&self) -> String {
        unsafe {
            let ptr = wb_robot_get_custom_data();
            if ptr.is_null() {
                String::new()
            } else {
                CStr::from_ptr(ptr).to_string_lossy().into_owned()
            }
        }
    }
    pub fn get_device(&self, name: &str) -> Option<WbDeviceTag> {
        let c_name = CString::new(name).ok()?;
        let tag = unsafe { wb_robot_get_device(c_name.as_ptr()) };
        if tag == 0 { None } else { Some(tag) }
    }
}

impl Drop for Robot {
    fn drop(&mut self) {
        unsafe { wb_robot_cleanup(); }
    }
}

#[derive(Clone, Copy)]
pub struct Motor(pub WbDeviceTag);
impl Motor {
    pub fn set_position(&self, pos: f64) { unsafe { wb_motor_set_position(self.0, pos); } }
    pub fn set_velocity(&self, vel: f64) { unsafe { wb_motor_set_velocity(self.0, vel); } }
}

#[derive(Clone, Copy)]
pub struct Gps(pub WbDeviceTag);
impl Gps {
    pub fn enable(&self, ms: i32) { unsafe { wb_gps_enable(self.0, ms); } }
    pub fn values(&self) -> [f64; 3] {
        unsafe {
            let p = wb_gps_get_values(self.0);
            [*p, *p.add(1), *p.add(2)]
        }
    }
    pub fn speed_vector(&self) -> [f64; 3] {
        unsafe {
            let p = wb_gps_get_speed_vector(self.0);
            [*p, *p.add(1), *p.add(2)]
        }
    }
}

#[derive(Clone, Copy)]
pub struct InertialUnit(pub WbDeviceTag);
impl InertialUnit {
    pub fn enable(&self, ms: i32) { unsafe { wb_inertial_unit_enable(self.0, ms); } }
    pub fn roll_pitch_yaw(&self) -> [f64; 3] {
        unsafe {
            let p = wb_inertial_unit_get_roll_pitch_yaw(self.0);
            [*p, *p.add(1), *p.add(2)]
        }
    }
}

#[derive(Clone, Copy)]
pub struct Gyro(pub WbDeviceTag);
impl Gyro {
    pub fn enable(&self, ms: i32) { unsafe { wb_gyro_enable(self.0, ms); } }
    pub fn values(&self) -> [f64; 3] {
        unsafe {
            let p = wb_gyro_get_values(self.0);
            [*p, *p.add(1), *p.add(2)]
        }
    }
}

#[derive(Clone, Copy)]
pub struct Camera {
    pub tag: WbDeviceTag,
    pub width: usize,
    pub height: usize,
}
impl Camera {
    pub fn enable(tag: WbDeviceTag, ms: i32) -> Self {
        unsafe {
            wb_camera_enable(tag, ms);
            let width = wb_camera_get_width(tag) as usize;
            let height = wb_camera_get_height(tag) as usize;
            Self { tag, width, height }
        }
    }
    pub fn bgra_slice(&self) -> &[u8] {
        unsafe {
            let ptr = wb_camera_get_image(self.tag);
            if ptr.is_null() {
                &[]
            } else {
                slice::from_raw_parts(ptr, self.width * self.height * 4)
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct DistanceSensor(pub WbDeviceTag);
impl DistanceSensor {
    pub fn enable(&self, ms: i32) { unsafe { wb_distance_sensor_enable(self.0, ms); } }
    pub fn value(&self) -> f64 { unsafe { wb_distance_sensor_get_value(self.0) } }
}

#[derive(Clone, Copy)]
pub struct Emitter(pub WbDeviceTag);
impl Emitter {
    pub fn send(&self, data: &[u8]) -> i32 {
        unsafe { wb_emitter_send(self.0, data.as_ptr() as *const c_void, data.len() as c_int) }
    }
}

#[derive(Clone, Copy)]
pub struct Receiver(pub WbDeviceTag);
impl Receiver {
    pub fn enable(&self, ms: i32) { unsafe { wb_receiver_enable(self.0, ms); } }
    pub fn queue_length(&self) -> i32 { unsafe { wb_receiver_get_queue_length(self.0) } }
    pub fn pop_bytes(&self) -> Option<Vec<u8>> {
        unsafe {
            if wb_receiver_get_queue_length(self.0) <= 0 {
                return None;
            }
            let ptr = wb_receiver_get_data(self.0) as *const u8;
            let len = wb_receiver_get_data_size(self.0) as usize;
            let out = slice::from_raw_parts(ptr, len).to_vec();
            wb_receiver_next_packet(self.0);
            Some(out)
        }
    }
}

pub struct Supervisor;
impl Supervisor {
    pub fn from_def(def: &str) -> Option<WbNodeRef> {
        let c_def = CString::new(def).ok()?;
        let r = unsafe { wb_supervisor_node_get_from_def(c_def.as_ptr()) };
        if r.is_null() { None } else { Some(r) }
    }
    pub fn node_position(node: WbNodeRef) -> [f64; 3] {
        unsafe {
            let p = wb_supervisor_node_get_position(node);
            [*p, *p.add(1), *p.add(2)]
        }
    }
    pub fn node_field(node: WbNodeRef, field_name: &str) -> Option<WbFieldRef> {
        let c_name = CString::new(field_name).ok()?;
        let f = unsafe { wb_supervisor_node_get_field(node, c_name.as_ptr()) };
        if f.is_null() { None } else { Some(f) }
    }
    pub fn field_get_vec3f(field: WbFieldRef) -> [f64; 3] {
        unsafe {
            let p = wb_supervisor_field_get_sf_vec3f(field);
            [*p, *p.add(1), *p.add(2)]
        }
    }
    pub fn field_set_vec3f(field: WbFieldRef, vals: [f64; 3]) {
        unsafe { wb_supervisor_field_set_sf_vec3f(field, vals.as_ptr()); }
    }
    pub fn add_force(node: WbNodeRef, force: [f64; 3], relative: bool) {
        unsafe { wb_supervisor_node_add_force(node, force.as_ptr(), if relative { 1 } else { 0 }); }
    }
    pub fn quit(status: i32) {
        unsafe { wb_supervisor_simulation_quit(status as c_int); }
    }
    pub fn dump_root_scene() -> Vec<String> {
        unsafe {
            let root = wb_supervisor_node_get_root();
            let f_children = CString::new("children").unwrap();
            let f_name = CString::new("name").unwrap();
            let field = wb_supervisor_node_get_field(root, f_children.as_ptr());
            let count = wb_supervisor_field_get_count(field);
            let mut out = Vec::with_capacity(count as usize);
            for i in 0..count {
                let n = wb_supervisor_field_get_mf_node(field, i);
                let tname = CStr::from_ptr(wb_supervisor_node_get_type_name(n)).to_string_lossy();
                let nf = wb_supervisor_node_get_field(n, f_name.as_ptr());
                let nstr = if nf.is_null() {
                    String::new()
                } else {
                    CStr::from_ptr(wb_supervisor_field_get_sf_string(nf)).to_string_lossy().into_owned()
                };
                let p = Self::node_position(n);
                out.push(format!("{tname} name={nstr} pos=[{:.4}, {:.4}, {:.4}]", p[0], p[1], p[2]));
            }
            out
        }
    }
}
