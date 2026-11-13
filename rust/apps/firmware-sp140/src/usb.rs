use crate::{
    app::{Commands, Request, Shared},
    diagnostics::Diagnostics,
    platform,
    storage::{self, RadioSettings},
};
use serde_json::{json, Value};
use std::io::{self, Write};
use std::sync::{Arc, Mutex};
struct UsbWriter;
impl Write for UsbWriter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let n = unsafe { platform::eppg_usb_write(bytes.as_ptr(), bytes.len()) };
        if n > 0 {
            Ok(n as usize)
        } else {
            Err(io::ErrorKind::WouldBlock.into())
        }
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}
fn send_diagnostics(packet: ([u8; 92], Vec<[u8; 92]>)) {
    let mut out = io::BufWriter::with_capacity(256, UsbWriter);
    if crate::diagnostic_json::write_json(&mut out, &packet.0, &packet.1).is_ok() {
        let _ = out.write_all(b"\n");
        let _ = out.flush();
    }
}
pub fn sync(shared: &Shared, radio: &RadioSettings) -> Value {
    let s = shared.lock().unwrap();
    let d = &s.settings;
    let mut mac = [0; 6];
    unsafe {
        esp_idf_svc::sys::esp_read_mac(
            mac.as_mut_ptr(),
            esp_idf_svc::sys::esp_mac_type_t_ESP_MAC_WIFI_STA,
        );
    }
    json!({"mj_v":8,"mi_v":1,"arch":"ESP32S3","scr_rt":d.screen_rotation,"ar_tme":d.armed_time,"m_tmp":d.metric_temp,"m_alt":d.metric_alt,"prf":d.performance_mode,"sea_p":d.sea_pressure,"thm":d.theme,"wireless_role":radio.name(),"wireless_mac":storage::mac_text(&mac),"wireless_peer":radio.peer.map(|p|storage::mac_text(&p)).unwrap_or_default(),"wireless_channel":radio.channel,"wireless_key_set":radio.secret.is_some()})
}
fn send(v: Value) {
    platform::console(&format!("{v}\n"));
}
pub fn spawn(
    shared: Shared,
    commands: Commands,
    mut radio: RadioSettings,
    diagnostics: Arc<Mutex<Diagnostics>>,
) {
    std::thread::Builder::new()
        .name("usb-config".into())
        .stack_size(7000)
        .spawn(move || {
            let mut line = [0u8; 256];
            let mut len = 0;
            let mut overflow = false;
            loop {
                let mut bytes = [0u8; 64];
                let n = unsafe { platform::eppg_usb_read(bytes.as_mut_ptr(), bytes.len()) };
                if n > 0 {
                    for byte in &bytes[..n as usize] {
                        if *byte == b'\r' {
                            continue;
                        }
                        if overflow {
                            if *byte == b'\n' {
                                overflow = false;
                                len = 0;
                            }
                            continue;
                        }
                        if *byte == b'\n' {
                            if len > 0 {
                                if let Ok(v) = serde_json::from_slice::<Value>(&line[..len]) {
                                    handle(v, &shared, &commands, &mut radio, &diagnostics);
                                }
                                len = 0;
                            }
                        } else if len + 1 >= line.len() {
                            len = 0;
                            overflow = true;
                        } else {
                            line[len] = *byte;
                            len += 1;
                        }
                    }
                }
                diagnostics.lock().unwrap().snapshot(&shared);
                platform::sleep(10);
            }
        })
        .expect("USB configuration task");
}
fn handle(
    v: Value,
    shared: &Shared,
    commands: &Commands,
    radio: &mut RadioSettings,
    diagnostics: &Arc<Mutex<Diagnostics>>,
) {
    match v.get("command").and_then(Value::as_str) {
        Some("reboot") => {
            diagnostics.lock().unwrap().planned_restart(shared, 1);
            platform::console("Rebooting\n");
            unsafe {
                esp_idf_svc::sys::esp_restart();
            }
        }
        Some("sync") => {
            send(sync(shared, radio));
            return;
        }
        Some("diag_sync") => {
            let packet = diagnostics.lock().unwrap().packet();
            send_diagnostics(packet);
            return;
        }
        Some("diag_clear") => {
            let packet = {
                let mut d = diagnostics.lock().unwrap();
                d.clear();
                d.packet()
            };
            send_diagnostics(packet);
            return;
        }
        _ => {}
    }
    if let Some(settings) = v.get("settings") {
        if settings.is_object() {
            let mut d = shared.lock().unwrap().settings.clone();
            if let Some(n) = settings.get("screen_rot").and_then(Value::as_u64) {
                d.screen_rotation = n as u8;
            }
            if let Some(n) = settings.get("sea_pressure").and_then(Value::as_f64) {
                d.sea_pressure = n as f32;
            }
            if let Some(n) = settings.get("metric_temp").and_then(Value::as_bool) {
                d.metric_temp = n;
            }
            if let Some(n) = settings.get("metric_alt").and_then(Value::as_bool) {
                d.metric_alt = n;
            }
            if let Some(n) = settings.get("performance_mode").and_then(Value::as_u64) {
                d.performance_mode = n as u8;
            }
            if let Some(n) = settings.get("theme").and_then(Value::as_u64) {
                d.theme = n as u8;
            }
            d.sanitize();
            if storage::save_settings(&d).is_ok() {
                shared.lock().unwrap().settings = d.clone();
                let _ = commands.try_send(Request::Settings(d));
            }
            send(sync(shared, radio));
        }
    }
    if let Some(wireless) = v.get("wireless") {
        if wireless.is_object() && radio.update(wireless) {
            platform::console(
                "Wireless settings saved; reboot required to apply role/radio changes\n",
            );
            send(sync(shared, radio));
        } else {
            platform::console("Invalid wireless settings\n");
        }
    }
}
