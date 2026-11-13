use crate::platform;
use std::ffi::c_void;
unsafe extern "C" {
    fn eppg_bmp_new() -> *mut c_void;
    fn eppg_bmp_free(p: *mut c_void);
    fn eppg_bmp_config(p: *mut c_void, temp_os: u8, pressure_os: u8, iir: u8, odr: u8) -> i32;
    fn eppg_bmp_sample(p: *mut c_void, temp: *mut f64, pressure: *mut f64) -> i32;
}
pub struct Altimeter {
    sensor: *mut c_void,
    pub ground: f32,
    pub altitude: f32,
    pub temperature: f32,
    pub pressure: f32,
    pub vertical: f32,
    history: [(f32, u32); 50],
    len: usize,
    next: usize,
}
impl Altimeter {
    pub fn new() -> Option<Self> {
        platform::sleep(20);
        let mut sensor = std::ptr::null_mut();
        for _ in 0..4 {
            sensor = unsafe { eppg_bmp_new() };
            if !sensor.is_null() {
                break;
            }
            platform::sleep(50);
        }
        if sensor.is_null() {
            return None;
        }
        if unsafe { eppg_bmp_config(sensor, 1, 2, 4, 3) } != 0 {
            unsafe {
                eppg_bmp_free(sensor);
            }
            return None;
        }
        let mut s = Self {
            sensor,
            ground: 0.,
            altitude: 0.,
            temperature: f32::MIN_POSITIVE,
            pressure: 0.,
            vertical: 0.,
            history: [(0., 0); 50],
            len: 0,
            next: 0,
        };
        s.sample(0, 1013.25);
        Some(s)
    }
    pub fn sample(&mut self, now: u32, sea: f32) {
        let (mut temperature, mut pressure) = (0., 0.);
        if unsafe { eppg_bmp_sample(self.sensor, &mut temperature, &mut pressure) } != 0 {
            return;
        }
        self.temperature = temperature as f32;
        self.pressure = (pressure / 100.) as f32;
        self.altitude = (44330.0 * (1.0 - libm::pow((self.pressure / sea) as f64, 0.1903))) as f32
            - self.ground;
        self.history[self.next] = (self.altitude, now);
        self.next = (self.next + 1) % 50;
        self.len = (self.len + 1).min(50);
        if self.len >= 2 {
            let oldest = self.history[if self.len == 50 { self.next } else { 0 }];
            let dt = now.wrapping_sub(oldest.1) as f32 / 1000.;
            self.vertical = if dt > 0. {
                ((self.altitude - oldest.0) / dt).clamp(-250., 250.)
            } else {
                0.
            };
        }
    }
    pub fn zero(&mut self) {
        self.ground += self.altitude;
        self.altitude = 0.;
        self.len = 0;
        self.next = 0;
        self.vertical = 0.;
    }
}
impl Drop for Altimeter {
    fn drop(&mut self) {
        unsafe {
            eppg_bmp_free(self.sensor);
        }
    }
}
