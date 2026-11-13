//! Raw NVS storage preserves the Arduino Preferences key/type ABI.
use controller_core::{Role, Settings};
use esp_idf_svc::sys;
use std::ffi::CString;
pub struct Nvs {
    handle: sys::nvs_handle_t,
}
impl Nvs {
    pub fn open(namespace: &str) -> Result<Self, i32> {
        let name = CString::new(namespace).map_err(|_| sys::ESP_ERR_INVALID_ARG)?;
        let mut handle = 0;
        let e = unsafe {
            sys::nvs_open(
                name.as_ptr(),
                sys::nvs_open_mode_t_NVS_READWRITE,
                &mut handle,
            )
        };
        if e == 0 {
            Ok(Self { handle })
        } else {
            Err(e)
        }
    }
    pub fn u8(&self, key: &str, default: u8) -> u8 {
        let mut v = default;
        if let Ok(k) = CString::new(key) {
            unsafe {
                sys::nvs_get_u8(self.handle, k.as_ptr(), &mut v);
            }
        }
        v
    }
    pub fn u16(&self, key: &str, default: u16) -> u16 {
        let mut v = default;
        if let Ok(k) = CString::new(key) {
            unsafe {
                sys::nvs_get_u16(self.handle, k.as_ptr(), &mut v);
            }
        }
        v
    }
    pub fn i32(&self, key: &str, default: i32) -> i32 {
        let mut v = default;
        if let Ok(k) = CString::new(key) {
            unsafe {
                sys::nvs_get_i32(self.handle, k.as_ptr(), &mut v);
            }
        }
        v
    }
    pub fn blob(&self, key: &str) -> Option<Vec<u8>> {
        let k = CString::new(key).ok()?;
        let mut len = 0;
        let err =
            unsafe { sys::nvs_get_blob(self.handle, k.as_ptr(), std::ptr::null_mut(), &mut len) };
        if err != 0 || len > 4096 {
            return None;
        }
        let mut b = vec![0; len];
        if unsafe { sys::nvs_get_blob(self.handle, k.as_ptr(), b.as_mut_ptr().cast(), &mut len) }
            == 0
        {
            Some(b)
        } else {
            None
        }
    }
    pub fn set_u8(&mut self, key: &str, v: u8) -> Result<(), i32> {
        let k = CString::new(key).map_err(|_| sys::ESP_ERR_INVALID_ARG)?;
        check(unsafe { sys::nvs_set_u8(self.handle, k.as_ptr(), v) })
    }
    pub fn set_u16(&mut self, key: &str, v: u16) -> Result<(), i32> {
        let k = CString::new(key).map_err(|_| sys::ESP_ERR_INVALID_ARG)?;
        check(unsafe { sys::nvs_set_u16(self.handle, k.as_ptr(), v) })
    }
    pub fn set_i32(&mut self, key: &str, v: i32) -> Result<(), i32> {
        let k = CString::new(key).map_err(|_| sys::ESP_ERR_INVALID_ARG)?;
        check(unsafe { sys::nvs_set_i32(self.handle, k.as_ptr(), v) })
    }
    pub fn set_blob(&mut self, key: &str, v: &[u8]) -> Result<(), i32> {
        let k = CString::new(key).map_err(|_| sys::ESP_ERR_INVALID_ARG)?;
        check(unsafe { sys::nvs_set_blob(self.handle, k.as_ptr(), v.as_ptr().cast(), v.len()) })
    }
    pub fn remove(&mut self, key: &str) -> Result<(), i32> {
        let k = CString::new(key).map_err(|_| sys::ESP_ERR_INVALID_ARG)?;
        let e = unsafe { sys::nvs_erase_key(self.handle, k.as_ptr()) };
        if e == sys::ESP_ERR_NVS_NOT_FOUND {
            Ok(())
        } else {
            check(e)
        }
    }
    pub fn clear(&mut self) -> Result<(), i32> {
        check(unsafe { sys::nvs_erase_all(self.handle) })
    }
    pub fn commit(&mut self) -> Result<(), i32> {
        check(unsafe { sys::nvs_commit(self.handle) })
    }
}
impl Drop for Nvs {
    fn drop(&mut self) {
        unsafe {
            sys::nvs_close(self.handle);
        }
    }
}
fn check(e: i32) -> Result<(), i32> {
    if e == 0 {
        Ok(())
    } else {
        Err(e)
    }
}
pub fn load_settings() -> Settings {
    let mut s = Settings::default();
    if let Ok(n) = Nvs::open("openppg") {
        s.version_major = n.u8("ver_major", s.version_major);
        s.version_minor = n.u8("ver_minor", s.version_minor);
        s.screen_rotation = n.u8("scr_rot", 3);
        if let Some(b) = n.blob("sea_pres") {
            if let Ok(v) = b.try_into() {
                s.sea_pressure = f32::from_le_bytes(v);
            }
        }
        s.metric_temp = n.u8("metric_tmp", 1) != 0;
        s.metric_alt = n.u8("metric_alt", 1) != 0;
        s.performance_mode = n.u8("perf_mode", 0);
        s.theme = n.u8("theme", 0);
        s.armed_time = n.u16("armed_time", 0);
        s.revision = n.u8("revision", 0);
        s.timezone_offset = n.i32("tz_offset", 0);
    }
    s.sanitize();
    s
}
static SETTINGS_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
pub fn save_settings(s: &Settings) -> Result<(), i32> {
    let _lock = SETTINGS_LOCK.lock().unwrap();
    let mut n = Nvs::open("openppg")?;
    n.set_u8("ver_major", s.version_major)?;
    n.set_u8("ver_minor", s.version_minor)?;
    n.set_u8("scr_rot", s.screen_rotation)?;
    n.set_blob("sea_pres", &s.sea_pressure.to_le_bytes())?;
    n.set_u8("metric_tmp", s.metric_temp as u8)?;
    n.set_u8("metric_alt", s.metric_alt as u8)?;
    n.set_u8("perf_mode", s.performance_mode)?;
    n.set_u8("theme", s.theme)?;
    n.set_u16("armed_time", s.armed_time)?;
    n.set_u8("revision", s.revision)?;
    n.set_i32("tz_offset", s.timezone_offset)?;
    n.commit()
}
#[derive(Clone)]
pub struct RadioSettings {
    pub role: Role,
    pub channel: u8,
    pub peer: Option<[u8; 6]>,
    pub secret: Option<[u8; 16]>,
}
impl Default for RadioSettings {
    fn default() -> Self {
        Self {
            role: Role::Standalone,
            channel: 6,
            peer: None,
            secret: None,
        }
    }
}
impl RadioSettings {
    pub fn load() -> Self {
        let mut s = Self::default();
        if let Ok(n) = Nvs::open("openppg-radio") {
            s.role = match n.u8("role", 0) {
                1 => Role::Remote,
                2 => Role::Receiver,
                _ => Role::Standalone,
            };
            s.channel = n.u8("channel", 6);
            if !(1..=13).contains(&s.channel) {
                s.channel = 6;
            }
            s.peer = n.blob("peer").and_then(|v| v.try_into().ok());
            s.secret = n.blob("secret").and_then(|v| v.try_into().ok());
        }
        s
    }
    pub fn name(&self) -> &'static str {
        match self.role {
            Role::Standalone => "disabled",
            Role::Remote => "remote",
            Role::Receiver => "receiver",
        }
    }
    pub fn save(&self) -> Result<(), i32> {
        let mut n = Nvs::open("openppg-radio")?;
        n.set_u8("role", self.role as u8)?;
        n.set_u8("channel", self.channel)?;
        if let Some(p) = self.peer {
            n.set_blob("peer", &p)?;
        } else {
            n.remove("peer")?;
        }
        if let Some(s) = self.secret {
            n.set_blob("secret", &s)?;
        } else {
            n.remove("secret")?;
        }
        n.commit()
    }
    pub fn update(&mut self, json: &serde_json::Value) -> bool {
        let mut next = self.clone();
        if let Some(role) = json.get("role") {
            next.role = match role.as_str() {
                Some("disabled") => Role::Standalone,
                Some("remote") => Role::Remote,
                Some("receiver") => Role::Receiver,
                _ => return false,
            };
        }
        if let Some(channel) = json.get("channel") {
            let Some(c) = channel.as_u64() else {
                return false;
            };
            if !(1..=13).contains(&c) {
                return false;
            }
            next.channel = c as u8;
        }
        if let Some(peer) = json.get("peer") {
            next.peer = peer.as_str().and_then(hex::<6>);
            if next.peer.is_none() {
                return false;
            }
        }
        if let Some(secret) = json.get("key") {
            next.secret = secret.as_str().and_then(hex::<16>);
            if next.secret.is_none() {
                return false;
            }
        }
        if next.role != Role::Standalone && (next.peer.is_none() || next.secret.is_none()) {
            return false;
        }
        if next.save().is_err() {
            return false;
        }
        *self = next;
        true
    }
}
fn hex<const N: usize>(s: &str) -> Option<[u8; N]> {
    let mut out = [0; N];
    let mut count = 0;
    let mut high = None;
    for b in s.bytes() {
        if N == 6 && (b == b':' || b == b'-') {
            continue;
        }
        let v = (b as char).to_digit(16)? as u8;
        if let Some(h) = high.take() {
            if count >= N {
                return None;
            }
            out[count] = h * 16 + v;
            count += 1;
        } else {
            high = Some(v);
        }
    }
    if count == N && high.is_none() {
        Some(out)
    } else {
        None
    }
}
pub fn mac_text(mac: &[u8; 6]) -> String {
    format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    )
}
