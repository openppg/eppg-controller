//! Native Rust GATT services and bonded pairing/reconnect policy.
use crate::{
    app::{Commands, Request, Shared},
    ble_ids::*,
    platform, storage,
};
use controller_core::State;
use esp32_nimble::{
    enums::*,
    utilities::{mutex::Mutex as NimbleMutex, BleUuid},
    BLEAdvertisementData, BLECharacteristic, BLEDevice, NimbleProperties as P,
};
use esp_idf_svc::sys;
use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicU32, Ordering},
        mpsc::{sync_channel, TryRecvError},
        Arc,
    },
};
type Characteristic = Arc<NimbleMutex<BLECharacteristic>>;
#[derive(Debug)]
enum Event {
    Write(&'static str, Vec<u8>),
    Connected(u16),
    Disconnected,
    Authenticated(bool, u16, esp32_nimble::BLEAddress),
    Pair,
}
pub struct Ble {
    pub pair: std::sync::mpsc::SyncSender<()>,
}
fn uuid(text: &str) -> BleUuid {
    if text.len() == 4 {
        BleUuid::from_uuid16(u16::from_str_radix(text, 16).unwrap())
    } else {
        BleUuid::from_uuid128_string(text).unwrap()
    }
}
fn secure_rw() -> P {
    P::READ | P::WRITE | P::READ_ENC | P::WRITE_ENC
}
fn read_value(id: &str, shared: &Shared, offset: u32) -> Vec<u8> {
    let s = shared.lock().unwrap();
    let d = &s.settings;
    match id {
        METRIC_ALT_UUID => vec![d.metric_alt as u8],
        METRIC_TEMP_UUID => vec![d.metric_temp as u8],
        PERFORMANCE_MODE_UUID => vec![d.performance_mode],
        SCREEN_ROTATION_UUID => vec![d.screen_rotation],
        THEME_UUID => vec![d.theme],
        SEA_PRESSURE_UUID => d.sea_pressure.to_le_bytes().to_vec(),
        TIMEZONE_UUID => d.timezone_offset.to_le_bytes().to_vec(),
        ARMED_TIME_UUID => d.armed_time.to_le_bytes().to_vec(),
        HW_REVISION_UUID => vec![d.revision],
        FW_VERSION_UUID => {
            let mut v = vec![8, 1];
            v.extend_from_slice(&crate::VERSION_BUILD.to_le_bytes());
            v
        }
        DEVICE_STATE_UUID => vec![s.state as u8],
        THROTTLE_VALUE_UUID => s.telemetry.pot_raw.to_le_bytes().to_vec(),
        FAST_LINK_TELEMETRY_UUID => s.telemetry.encode().to_vec(),
        ESC_RELAY_STATUS_UUID => s.esc_status.clone(),
        ESC_FW_CTRL_UUID => s.esc_fw_status.to_vec(),
        ESC_PARAM_DATA_UUID => {
            let at = offset as usize;
            if at >= s.esc_result.len() {
                vec![]
            } else {
                s.esc_result[at..(at + 240).min(s.esc_result.len())].to_vec()
            }
        }
        UNIX_TIME_UUID => {
            let mut now = 0;
            unsafe {
                sys::time(&mut now);
            }
            ((now - d.timezone_offset as i64) as i32)
                .to_le_bytes()
                .to_vec()
        }
        _ => vec![],
    }
}
fn apply_setting(id: &str, data: &[u8], shared: &Shared, commands: &Commands) -> bool {
    let mut d = shared.lock().unwrap().settings.clone();
    match id {
        METRIC_ALT_UUID if data.len() == 1 => d.metric_alt = data[0] != 0,
        METRIC_TEMP_UUID if data.len() == 1 => d.metric_temp = data[0] != 0,
        PERFORMANCE_MODE_UUID if data.len() == 1 && data[0] <= 1 => d.performance_mode = data[0],
        SCREEN_ROTATION_UUID if data.len() == 1 && matches!(data[0], 1 | 3) => {
            d.screen_rotation = data[0]
        }
        THEME_UUID if data.len() == 1 && data[0] <= 1 => d.theme = data[0],
        SEA_PRESSURE_UUID if data.len() == 4 => {
            let v = f32::from_le_bytes(data.try_into().unwrap());
            if v < 300.0 || v > 1200.0 {
                return false;
            }
            d.sea_pressure = v;
        }
        TIMEZONE_UUID if data.len() == 4 => {
            d.timezone_offset = i32::from_le_bytes(data.try_into().unwrap())
        }
        UNIX_TIME_UUID if data.len() == 4 => {
            let t = i32::from_le_bytes(data.try_into().unwrap()) as i64 + d.timezone_offset as i64;
            let tv = sys::timeval {
                tv_sec: t,
                tv_usec: 0,
            };
            unsafe {
                sys::settimeofday(&tv, std::ptr::null());
            }
            return true;
        }
        _ => return false,
    }
    if storage::save_settings(&d).is_err() {
        return false;
    }
    shared.lock().unwrap().settings = d.clone();
    let _ = commands.try_send(Request::Settings(d));
    true
}
#[cfg(feature = "qemu")]
pub fn spawn(shared: Shared, _commands: Commands) -> Ble {
    let (pair, pending) = sync_channel(1);
    platform::console(
        "EPPG_QEMU BLE hardware unsupported; service boundary modeled by native Rust lab\n",
    );
    std::thread::spawn(move || {
        while pending.recv().is_ok() {
            shared.lock().unwrap().pairing = true;
            platform::sleep(60000);
            shared.lock().unwrap().pairing = false;
        }
    });
    Ble { pair }
}
#[cfg(not(feature = "qemu"))]
pub fn spawn(shared: Shared, commands: Commands) -> Ble {
    let (pair_tx, pair_rx) = sync_channel(1);
    std::thread::Builder::new()
        .name("ble-services".into())
        .stack_size(10000)
        .spawn(move || {
            let device = BLEDevice::take();
            device
                .security()
                .set_auth(AuthReq::Bond | AuthReq::Sc)
                .set_io_cap(SecurityIOCap::NoInputNoOutput)
                .set_security_init_key(PairKeyDist::ENC | PairKeyDist::ID)
                .set_security_resp_key(PairKeyDist::ENC | PairKeyDist::ID)
                .resolve_rpa();
            let _ = device.set_preferred_mtu(517);
            let _ = device.set_power(PowerType::Default, PowerLevel::P9);
            let mut mac = [0; 6];
            unsafe {
                sys::esp_read_mac(mac.as_mut_ptr(), sys::esp_mac_type_t_ESP_MAC_BT);
            }
            let name = format!("OpenPPG SP140 [{:02X}{:02X}]", mac[4], mac[5]);
            let _ = BLEDevice::set_device_name(&name);
            let unique = storage::mac_text(&mac);
            let (tx, rx) = sync_channel(96);
            let server = device.get_server();
            server.advertise_on_disconnect(false);
            let out = tx.clone();
            server.on_connect(move |_, desc| {
                let _ = out.try_send(Event::Connected(desc.conn_handle()));
            });
            let out = tx.clone();
            server.on_disconnect(move |_, _| {
                let _ = out.try_send(Event::Disconnected);
            });
            let out = tx.clone();
            server.on_authentication_complete(move |server, desc, result| {
                let ok = result.is_ok() && desc.bonded() && desc.encrypted();
                let _ = out.try_send(Event::Authenticated(
                    ok,
                    desc.conn_handle(),
                    desc.id_address(),
                ));
                let _ = server;
            });
            let mut chars: HashMap<&'static str, Characteristic> = HashMap::new();
            let offset = Arc::new(AtomicU32::new(0));
            let config = server.create_service(uuid(CONFIG_SERVICE_UUID));
            let config_defs = [
                (UNIX_TIME_UUID, secure_rw()),
                (TIMEZONE_UUID, secure_rw()),
                (DEVICE_STATE_UUID, P::READ | P::NOTIFY | P::READ_ENC),
                (METRIC_ALT_UUID, secure_rw()),
                (PERFORMANCE_MODE_UUID, secure_rw()),
                (SCREEN_ROTATION_UUID, secure_rw()),
                (THEME_UUID, secure_rw()),
                (SEA_PRESSURE_UUID, secure_rw()),
                (METRIC_TEMP_UUID, secure_rw()),
                (FW_VERSION_UUID, P::READ | P::READ_ENC),
                (HW_REVISION_UUID, P::READ | P::READ_ENC),
                (ARMED_TIME_UUID, P::READ | P::READ_ENC),
                (THROTTLE_VALUE_UUID, secure_rw() | P::NOTIFY | P::INDICATE),
                (ESC_RELAY_CMD_UUID, secure_rw()),
                (ESC_RELAY_STATUS_UUID, P::READ | P::READ_ENC),
                (ESC_FW_CTRL_UUID, secure_rw()),
                (ESC_FW_DATA_UUID, P::WRITE | P::WRITE_NO_RSP | P::WRITE_ENC),
                (ESC_PARAM_DATA_UUID, secure_rw()),
                (ESC_RELAY_NOTIFY_UUID, P::READ | P::NOTIFY | P::READ_ENC),
            ];
            for (id, properties) in config_defs {
                let c = config.lock().create_characteristic(uuid(id), properties);
                let source = shared.clone();
                let cursor = offset.clone();
                if !matches!(
                    id,
                    METRIC_ALT_UUID | PERFORMANCE_MODE_UUID | SCREEN_ROTATION_UUID
                ) {
                    c.lock().on_read(move |c, _| {
                        c.set_value(&read_value(id, &source, cursor.load(Ordering::Relaxed)));
                    });
                }
                let source = tx.clone();
                let cursor = offset.clone();
                c.lock().on_write(move |args| {
                    let data = args.recv_data();
                    if id == ESC_PARAM_DATA_UUID && data.len() >= 4 {
                        cursor.store(
                            u32::from_le_bytes(data[..4].try_into().unwrap()),
                            Ordering::Relaxed,
                        );
                    } else if source.try_send(Event::Write(id, data.to_vec())).is_err() {
                        args.reject();
                    }
                });
                let mut initial = read_value(id, &shared, 0);
                if matches!(
                    id,
                    METRIC_ALT_UUID | PERFORMANCE_MODE_UUID | SCREEN_ROTATION_UUID
                ) {
                    initial.resize(4, 0);
                }
                c.lock().set_value(&initial);
                chars.insert(id, c);
            }
            let info = server.create_service(uuid(DEVICE_INFO_SERVICE_UUID));
            info.lock()
                .create_characteristic(uuid(MANUFACTURER_NAME_UUID), P::READ | P::READ_ENC)
                .lock()
                .set_value(b"OpenPPG");
            info.lock()
                .create_characteristic(uuid(DEVICE_UNIQUE_ID_UUID), P::READ | P::READ_ENC)
                .lock()
                .set_value(unique.as_bytes());
            let fast = server.create_service(uuid(FAST_LINK_TELEMETRY_SERVICE_UUID));
            for (id, p) in [
                (FAST_LINK_TELEMETRY_UUID, P::READ | P::NOTIFY | P::READ_ENC),
                (FAST_LINK_COMMAND_UUID, P::WRITE_NO_RSP | P::WRITE_ENC),
            ] {
                let c = fast.lock().create_characteristic(uuid(id), p);
                if id == FAST_LINK_COMMAND_UUID {
                    let out = tx.clone();
                    c.lock().on_write(move |a| {
                        let _ = out.try_send(Event::Write(id, a.recv_data().to_vec()));
                    });
                }
                c.lock().set_value(&read_value(id, &shared, 0));
                chars.insert(id, c);
            }
            let ota_service = server.create_service(uuid(OTA_SERVICE_UUID));
            for (id, p) in [
                (
                    OTA_RECV_FW_UUID,
                    P::WRITE | P::WRITE_NO_RSP | P::NOTIFY | P::INDICATE,
                ),
                (OTA_PROGRESS_UUID, P::NOTIFY | P::INDICATE),
                (OTA_COMMAND_UUID, P::WRITE | P::NOTIFY | P::INDICATE),
                (OTA_CUSTOMER_UUID, P::NOTIFY | P::INDICATE),
            ] {
                let c = ota_service.lock().create_characteristic(uuid(id), p);
                let out = tx.clone();
                c.lock().on_write(move |a| {
                    if out
                        .try_send(Event::Write(id, a.recv_data().to_vec()))
                        .is_err()
                    {
                        a.reject();
                    }
                });
                chars.insert(id, c);
            }
            let _ = server.start();
            let adv = device.get_advertising();
            let _ = adv.lock().set_data(
                BLEAdvertisementData::new()
                    .name(&name)
                    .add_service_uuid(uuid(CONFIG_SERVICE_UUID)),
            );
            adv.lock()
                .scan_response(true)
                .min_interval(32)
                .max_interval(48);
            let mut connected = false;
            let mut handle = 0;
            let mut connected_at = 0;
            let mut tuned = false;
            let mut pairing_until = None;
            let mut last_adv = 0;
            let mut last_publish = 0;
            let mut last_keepalive = 0;
            let mut last_state = 255;
            let mut ota = crate::ota::Ota::default();
            let mut last_quiet = false;
            let mut stream_offset = 0usize;
            let mut stream_pass = 3;
            let mut terminal_resends = 0;
            let mut last_status = vec![];
            loop {
                let now = platform::now();
                if pair_rx.try_recv().is_ok() {
                    let _ = tx.try_send(Event::Pair);
                }
                for _ in 0..96 {
                    let event = match rx.try_recv() {
                        Ok(e) => e,
                        Err(TryRecvError::Empty) => break,
                        Err(_) => return,
                    };
                    match event {
                        Event::Pair => {
                            let _ = adv.lock().stop();
                            for c in server
                                .connections()
                                .map(|d| d.conn_handle())
                                .collect::<Vec<_>>()
                            {
                                let _ = server.disconnect(c);
                            }
                            let _ = device.delete_all_bonds();
                            let _ = device.set_white_list(&[]);
                            pairing_until = Some(now.wrapping_add(60000));
                            shared.lock().unwrap().pairing = true;
                        }
                        Event::Connected(h) => {
                            connected = true;
                            handle = h;
                            connected_at = now;
                            tuned = false;
                            shared.lock().unwrap().connected = true;
                        }
                        Event::Disconnected => {
                            connected = false;
                            ota.abort();
                            let mut s = shared.lock().unwrap();
                            s.connected = false;
                            s.ota = false;
                        }
                        Event::Authenticated(ok, h, address) => {
                            if ok {
                                pairing_until = None;
                                shared.lock().unwrap().pairing = false;
                            } else if pairing_until.is_some() {
                                let _ = device.delete_bond(&address);
                                unsafe {
                                    sys::ble_gap_security_initiate(h);
                                }
                            } else {
                                let _ = server.disconnect(h);
                            }
                        }
                        Event::Write(id, data) => {
                            if id == OTA_COMMAND_UUID {
                                let armed = {
                                    let mut s = shared.lock().unwrap();
                                    let armed = s.state != State::Disarmed;
                                    if !armed
                                        && matches!(
                                            controller_protocols::ota::parse_command(&data),
                                            Some(controller_protocols::ota::Command::Start(_))
                                        )
                                    {
                                        s.ota = true;
                                    }
                                    armed
                                };
                                let before = ota.active;
                                if let Some((ack, reboot)) = ota.command(&data, armed, now) {
                                    notify(&chars, OTA_COMMAND_UUID, &ack);
                                    if !before && ota.active {
                                        let _ = server.update_conn_params(handle, 12, 12, 0, 800);
                                    }
                                    if reboot {
                                        platform::sleep(1000);
                                        unsafe {
                                            sys::esp_restart();
                                        }
                                    }
                                }
                                shared.lock().unwrap().ota = ota.active;
                            } else if id == OTA_RECV_FW_UUID {
                                if let Some(ack) = ota.data(&data, now) {
                                    notify(&chars, OTA_RECV_FW_UUID, &ack);
                                }
                                shared.lock().unwrap().ota = ota.active;
                            } else if id == THROTTLE_VALUE_UUID && data.len() == 2 {
                                let _ =
                                    commands.try_send(Request::CruisePwm(u16::from_be_bytes([
                                        data[0], data[1],
                                    ])));
                            } else if id == FAST_LINK_COMMAND_UUID && data.first() == Some(&1) {
                                let _ = commands.try_send(Request::HardwareInfo);
                            } else if id == ESC_RELAY_CMD_UUID && !data.is_empty() {
                                let _ =
                                    commands.try_send(Request::Relay(data[0], data[1..].to_vec()));
                            } else if id == ESC_FW_CTRL_UUID && !data.is_empty() {
                                match data[0] {
                                    0x20 if data.len() >= 7 => {
                                        let hw = u16::from_le_bytes([data[1], data[2]]);
                                        let size =
                                            u32::from_le_bytes(data[3..7].try_into().unwrap());
                                        let accepted = {
                                            let mut s = shared.lock().unwrap();
                                            let code = if s.state != State::Disarmed {
                                                0x81
                                            } else if s.ota
                                                || s.relay
                                                || s.maintenance_pending
                                                || !(32..=262144).contains(&size)
                                            {
                                                0x84
                                            } else if hw == 0 || hw != s.telemetry.esc_hardware_id {
                                                0x83
                                            } else {
                                                0
                                            };
                                            if code == 0 {
                                                s.maintenance_pending = true;
                                                true
                                            } else {
                                                s.esc_fw_status = [code, 8, 0, 0];
                                                let _ =
                                                    commands.try_send(Request::FlashReject(code));
                                                false
                                            }
                                        };
                                        if accepted {
                                            if crate::staging::begin(size) {
                                                if commands
                                                    .send(Request::FlashBegin(hw, size))
                                                    .is_err()
                                                {
                                                    crate::staging::abort();
                                                    shared.lock().unwrap().maintenance_pending =
                                                        false;
                                                }
                                            } else {
                                                let mut s = shared.lock().unwrap();
                                                s.maintenance_pending = false;
                                                s.esc_fw_status = [0x84, 8, 0, 0];
                                                let _ =
                                                    commands.try_send(Request::FlashReject(0x84));
                                            }
                                        }
                                    }
                                    0x21 => {
                                        if crate::staging::finish() {
                                            let _ = commands.send(Request::FlashEnd);
                                        }
                                    }
                                    0x22 => {
                                        crate::staging::abort();
                                        let _ = commands.send(Request::FlashAbort);
                                    }
                                    _ => {}
                                }
                            } else if id == ESC_FW_DATA_UUID && data.len() >= 4 {
                                let offset = u32::from_le_bytes(data[..4].try_into().unwrap());
                                if crate::staging::write(offset, &data[4..]).is_some() {
                                    let _ = commands
                                        .send(Request::FlashProgress(offset, data.len() - 4));
                                }
                            } else {
                                apply_setting(id, &data, &shared, &commands);
                            }
                        }
                    }
                }
                crate::staging::pump_abort();
                if pairing_until
                    .map(|at| now.wrapping_sub(at) as i32 >= 0)
                    .unwrap_or(false)
                {
                    pairing_until = None;
                    shared.lock().unwrap().pairing = false;
                    let _ = adv.lock().stop();
                }
                if ota.timeout(now) {
                    shared.lock().unwrap().ota = false;
                    let _ = server.update_conn_params(handle, 24, 40, 0, 400);
                }
                if connected && !tuned && now.wrapping_sub(connected_at) >= 1200 {
                    let _ = server.update_conn_params(handle, 12, 24, 0, 200);
                    tuned = true;
                }
                if !connected && now.wrapping_sub(last_adv) >= 1000 {
                    last_adv = now;
                    if !adv.lock().is_advertising() {
                        let bonds = device.bonded_addresses().unwrap_or_default();
                        let _ = adv.lock().stop();
                        let _ = device.set_white_list(&bonds);
                        adv.lock().filter_policy(if pairing_until.is_some() {
                            AdvFilterPolicy::None
                        } else {
                            AdvFilterPolicy::Connect
                        });
                        if pairing_until.is_some() || !bonds.is_empty() {
                            let _ = adv.lock().start();
                        }
                    }
                }
                if now.wrapping_sub(last_publish) >= 20 {
                    last_publish = now;
                    let s = shared.lock().unwrap().clone();
                    let quiet = ota.active || s.transfer_quiet;
                    if last_quiet && !quiet {
                        let _ = server.update_conn_params(handle, 24, 40, 0, 400);
                    }
                    last_quiet = quiet;
                    if connected {
                        let bytes = s.telemetry.encode();
                        if !quiet || now.wrapping_sub(last_keepalive) >= 1000 {
                            notify(&chars, FAST_LINK_TELEMETRY_UUID, &bytes);
                            last_keepalive = now;
                        } else if let Some(c) = chars.get(FAST_LINK_TELEMETRY_UUID) {
                            c.lock().set_value(&bytes);
                        }
                        if last_state != s.state as u8 {
                            last_state = s.state as u8;
                            notify(&chars, DEVICE_STATE_UUID, &[last_state]);
                        }
                        let code = s.esc_status.first().copied().unwrap_or(0);
                        let phase = s.esc_status.get(3).copied().unwrap_or(0);
                        let detail = s.esc_status.get(4).copied().unwrap_or(0);
                        let status = vec![code, phase, detail];
                        let changed = status != last_status;
                        if changed || (terminal_resends > 0 && (code == 4 || code >= 0x80)) {
                            if changed {
                                terminal_resends = if code == 4 || code >= 0x80 { 4 } else { 0 };
                                last_status = status;
                            } else {
                                terminal_resends -= 1;
                            }
                            let len = if s.esc_status.len() >= 3 {
                                u16::from_le_bytes([s.esc_status[1], s.esc_status[2]])
                            } else {
                                0
                            };
                            notify(
                                &chars,
                                ESC_RELAY_NOTIFY_UUID,
                                &[1, code, phase, detail, len as u8, (len >> 8) as u8],
                            );
                            if code == 7 {
                                stream_offset = 0;
                                stream_pass = 0;
                            }
                        }
                        if code == 7 && stream_pass < 3 {
                            let total = s.esc_result.len();
                            if stream_offset >= total {
                                stream_offset = 0;
                                stream_pass += 1;
                            } else {
                                let n = (total - stream_offset).min(198);
                                let mut b = vec![
                                    2,
                                    stream_offset as u8,
                                    (stream_offset >> 8) as u8,
                                    total as u8,
                                    (total >> 8) as u8,
                                ];
                                b.extend_from_slice(
                                    &s.esc_result[stream_offset..stream_offset + n],
                                );
                                notify(&chars, ESC_RELAY_NOTIFY_UUID, &b);
                                stream_offset += n;
                            }
                        }
                    }
                }
                platform::sleep(5);
            }
        })
        .expect("BLE services task");
    Ble { pair: pair_tx }
}
fn notify(chars: &HashMap<&'static str, Characteristic>, id: &str, data: &[u8]) {
    if let Some(c) = chars.get(id) {
        c.lock().set_value(data).notify();
    }
}
