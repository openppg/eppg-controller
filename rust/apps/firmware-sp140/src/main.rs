//! Arduino-free ESP32-S3 application: controller, services and UI are native Rust.
#![cfg_attr(feature = "qemu", allow(dead_code, unused_imports))]
mod altimeter;
mod app;
mod ble;
mod ble_ids;
mod diagnostics;
mod feedback;
mod ota;
mod platform;
#[cfg(feature = "qemu")]
mod qemu;
mod staging;
mod storage;
mod usb;
mod wireless;
use app::{Request, Shared, Snapshot};
use controller_core::{
    monitor::{Data, Monitors},
    Controller, Effects, Input, Role, State,
};
use controller_esc::Esc;
use controller_protocols::{ant::AntBms, fastlink::FastLink};
use esp_idf_svc::{
    hal::{cpu::Core, task::thread::ThreadSpawnConfiguration},
    sys,
};
use std::{
    cell::RefCell,
    ffi::CStr,
    sync::{mpsc::sync_channel, Arc, Mutex},
};
// Retain the existing backend firmware-discovery marker through LTO/section GC.
include!(concat!(env!("OUT_DIR"), "/fw_marker.rs"));
thread_local! {static PANEL:RefCell<Option<platform::Panel>>=const{RefCell::new(None)};}
fn panel_flush(x1: i32, y1: i32, x2: i32, y2: i32, data: &[u8]) {
    PANEL.with(|p| {
        if let Some(p) = p.borrow_mut().as_mut() {
            p.flush(x1, y1, x2, y2, data);
        }
    });
}
fn configure_thread(name: &'static [u8], stack: usize, priority: u8, core: Core) {
    let c = ThreadSpawnConfiguration {
        name: Some(CStr::from_bytes_with_nul(name).unwrap()),
        stack_size: stack,
        priority,
        pin_to_core: Some(core),
        ..Default::default()
    };
    c.set().expect("task affinity/priority");
}
// std::thread::spawn initializes pthread attributes with IDF's 3 KiB default,
// overriding esp_pthread_cfg.stack_size. Always pass the configured size explicitly.
fn spawn_task<F, T>(f: F) -> std::thread::JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    let config = ThreadSpawnConfiguration::get().expect("configured thread");
    std::thread::Builder::new()
        .stack_size(config.stack_size)
        .spawn(f)
        .expect("task allocation")
}
fn periodic(last: &mut u32, ms: u32) {
    unsafe {
        sys::xTaskDelayUntil(last, ms);
    }
}
fn main() {
    sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    unsafe {
        let e = sys::nvs_flash_init();
        if e != 0 {
            panic!("NVS initialization failed {e}; preserving stored data");
        }
    }
    platform::init();
    platform::console("Booting OpenPPG SP140 Rust8.1\n");
    let settings = storage::load_settings();
    let radio_settings = storage::RadioSettings::load();
    let role = radio_settings.role;
    let shared: Shared = Arc::new(Mutex::new(Snapshot::new(settings.clone(), role)));
    let diagnostics = Arc::new(Mutex::new(diagnostics::Diagnostics::init(&shared)));
    let (commands, requests) = sync_channel(64);
    let watchdog = sys::esp_task_wdt_config_t {
        timeout_ms: 5000,
        idle_core_mask: 0,
        trigger_panic: true,
    };
    unsafe {
        let e = sys::esp_task_wdt_init(&watchdog);
        if e == sys::ESP_ERR_INVALID_STATE {
            sys::esp_task_wdt_reconfigure(&watchdog);
        }
    }
    configure_thread(b"feedback\0", 3000, 1, Core::Core1);
    let feedback = feedback::spawn(shared.clone());
    let _ = feedback.try_send(feedback::Feedback::Vibrate(400, 255));
    configure_thread(b"ble-services\0", 10000, 1, Core::Core1);
    let bluetooth = ble::spawn(shared.clone(), commands.clone());
    configure_thread(b"espnow\0", 6000, 1, Core::Core1);
    let remote = wireless::spawn(radio_settings.clone(), shared.clone(), commands.clone());
    configure_thread(b"usb-config\0", 7000, 1, Core::Core1);
    usb::spawn(
        shared.clone(),
        commands.clone(),
        radio_settings,
        diagnostics.clone(),
    );
    let (persist_tx, persist_rx) = sync_channel::<controller_core::Settings>(1);
    std::thread::Builder::new()
        .name("settings-writer".into())
        .stack_size(4000)
        .spawn(move || {
            while let Ok(settings) = persist_rx.recv() {
                if let Err(error) = storage::save_settings(&settings) {
                    platform::console(&format!("NVS save failed {error}\n"));
                }
            }
        })
        .expect("settings writer");
    let (zero_tx, zero_rx) = sync_channel::<()>(1);
    let (ui_tx, ui_rx) = sync_channel::<Effects>(10);
    configure_thread(b"sensors\0", 6000, 2, Core::Core1);
    if role != Role::Remote {
        let shared = shared.clone();
        crate::spawn_task(move || {
            let mut alt = altimeter::Altimeter::new();
            shared.lock().unwrap().baro_initialized = alt.is_some();
            let mut last = unsafe { sys::xTaskGetTickCount() };
            let mut last_cpu = 0u32.wrapping_sub(1000);
            loop {
                let now = platform::now();
                let sea = shared.lock().unwrap().settings.sea_pressure;
                if let Some(ref mut a) = alt {
                    while zero_rx.try_recv().is_ok() {
                        a.zero();
                    }
                    a.sample(now, sea);
                    let mut s = shared.lock().unwrap();
                    s.telemetry.altitude_cm = (a.altitude * 100.).round() as i32;
                    s.telemetry.baro_temp_dC = (a.temperature * 10.).round() as i16;
                    s.baro_temperature = a.temperature;
                    s.telemetry.baro_pressure_dHPa = (a.pressure * 10.).round() as u16;
                    s.telemetry.vario_cmps = (a.vertical * 100.).round() as i16;
                }
                if now.wrapping_sub(last_cpu) >= 1000 {
                    last_cpu = now;
                    let cpu = unsafe { platform::eppg_cpu_temperature() };
                    let mut s = shared.lock().unwrap();
                    s.cpu_temperature = cpu;
                    s.telemetry.mcu_temp_dC = (cpu * 10.).round() as i16;
                }
                periodic(&mut last, 100);
            }
        });
    }
    configure_thread(b"bms\0", 6000, 2, Core::Core1);
    if role != Role::Remote {
        let shared = shared.clone();
        crate::spawn_task(move || {
            #[cfg(feature = "qemu")]
            qemu::set_handle(2);
            let mut can = platform::Mcp2515::new();
            shared.lock().unwrap().bms_initialized = can.is_some();
            let mut bms = AntBms::default();
            let mut last = unsafe { sys::xTaskGetTickCount() };
            loop {
                let now = platform::now();
                if let Some(ref mut can) = can {
                    for _ in 0..128 {
                        if let Some((id, data, n)) = can.receive() {
                            bms.packet(id, &data[..n], now);
                        } else {
                            break;
                        }
                    }
                }
                let data = bms.snapshot(now);
                let temps = bms.temperatures();
                let mut s = shared.lock().unwrap();
                copy_bms(&mut s.telemetry, &data);
                s.bms_temperatures = temps;
                s.last_bms = now;
                drop(s);
                periodic(&mut last, 100);
            }
        });
    }
    configure_thread(b"ui\0", 16000, 2, Core::Core1);
    {
        let shared = shared.clone();
        let feedback = feedback.clone();
        crate::spawn_task(move || {
            let settings = shared.lock().unwrap().settings.clone();
            PANEL.with(|p| *p.borrow_mut() = platform::Panel::new(settings.screen_rotation));
            #[cfg(feature = "qemu")]
            qemu::set_handle(1);
            let mut ui = controller_ui::Ui::with_settings(&settings);
            ui.set_panel_flush(panel_flush);
            ui.show_splash(&settings, platform::now());
            #[cfg(feature = "qemu")]
            qemu::boot_complete();
            let mut last = unsafe { sys::xTaskGetTickCount() };
            let mut old_rotation = settings.screen_rotation;
            loop {
                let now = platform::now();
                let s = shared.lock().unwrap().clone();
                while let Ok(e) = ui_rx.try_recv() {
                    if e.contains(Effects::ARM_FAIL) {
                        ui.flash_arm_failure(now);
                    }
                    if e.contains(Effects::CRUISE_FAIL) {
                        ui.flash_cruise(now);
                    }
                    if e.contains(Effects::ARM) {
                        ui.set_armed_start(now);
                    }
                }
                let mut monitors = Monitors::default();
                monitors.levels = s.levels;
                ui.set_monitors(&monitors);
                if role != Role::Remote {
                    ui.set_battery_temperature(
                        s.bms_temperatures[2..6]
                            .iter()
                            .copied()
                            .filter(|t| t.is_finite())
                            .reduce(f32::max),
                    );
                }
                ui.set_connected(s.connected);
                if old_rotation != s.settings.screen_rotation {
                    old_rotation = s.settings.screen_rotation;
                    PANEL.with(|p| {
                        if let Some(p) = p.borrow_mut().as_mut() {
                            p.set_rotation(old_rotation);
                        }
                    });
                }
                ui.update(&s.telemetry, &s.settings, s.pairing, now);
                if ui.vibration_due {
                    let _ = feedback.try_send(feedback::Feedback::Vibrate(300, 200));
                }
                shared.lock().unwrap().last_ui = now;
                periodic(&mut last, 33);
            }
        });
    }
    configure_thread(b"control\0", 14336, 3, Core::Core0);
    {
        let shared = shared.clone();
        let feedback = feedback.clone();
        crate::spawn_task(move || {
            #[cfg(feature = "qemu")]
            qemu::set_handle(0);
            let mut controller = Controller::new(role);
            controller.mode = settings.performance_mode;
            controller.armed_minutes = settings.armed_time;
            let initialized = role != Role::Remote && platform::twai_init();
            shared.lock().unwrap().esc_initialized = initialized;
            let mut esc = if initialized { Esc::new() } else { None };
            if let Some(e) = esc.as_mut() {
                e.flasher =
                    controller_esc::Flasher::with_storage(Box::<staging::StagedImage>::default());
            }
            let mut monitors = Monitors::default();
            let mut last_monitor = 0;
            let mut old_warnings = 0;
            let mut last_health = 0;
            #[cfg(feature = "qemu")]
            let mut stack_reported = false;
            let mut last_mirrored = 0;
            let mut last = unsafe { sys::xTaskGetTickCount() };
            unsafe {
                sys::esp_task_wdt_add(std::ptr::null_mut());
            }
            loop {
                let now = platform::now();
                #[cfg(feature = "qemu")]
                if now >= 5000 && !stack_reported {
                    stack_reported = true;
                    platform::console(&format!("Rust control stack free_bytes={}\n", unsafe {
                        sys::uxTaskGetStackHighWaterMark(std::ptr::null_mut())
                    }));
                }
                let raw = unsafe { platform::eppg_adc_read(7) }.clamp(0, 4095) as u16;
                for _ in 0..64 {
                    let Ok(request) = requests.try_recv() else {
                        break;
                    };
                    match request {
                        Request::Settings(settings) => {
                            controller.mode = settings.performance_mode;
                            controller.armed_minutes = settings.armed_time;
                        }
                        Request::CruisePwm(pwm) => {
                            controller.set_cruise_pwm(pwm);
                        }
                        Request::Control(c) => {
                            controller.receive_control(
                                now,
                                c.session,
                                c.sequence,
                                c.throttle,
                                State::from_byte(c.state).unwrap_or(State::Disarmed),
                                c.mode,
                            );
                        }
                        Request::Ack(a) => controller.receive_ack(
                            now,
                            State::from_byte(a.state).unwrap_or(State::Disarmed),
                            a.flags & 2 != 0,
                        ),
                        Request::Mirrored(mut t) => {
                            last_mirrored = now;
                            t.pot_raw = raw;
                            let mut s = shared.lock().unwrap();
                            for i in 0..6 {
                                s.bms_temperatures[i] = t.bms_temp_sensors_C[i] as f32;
                            }
                            s.telemetry = t;
                        }
                        Request::HardwareInfo => {
                            if let Some(e) = esc.as_mut() {
                                e.request_hardware();
                            }
                        }
                        Request::Relay(op, data) => {
                            if let Some(e) = esc.as_mut() {
                                e.relay.command(op, &data, e.flasher.active());
                            }
                        }
                        Request::FlashBegin(hw, size) => {
                            if let Some(e) = esc.as_mut() {
                                if !e.flasher.begin(
                                    hw,
                                    size,
                                    e.telemetry.esc_hardware_id,
                                    controller.state,
                                    e.relay.active(),
                                    now,
                                ) {
                                    staging::abort();
                                }
                            }
                            shared.lock().unwrap().maintenance_pending = false;
                        }
                        Request::FlashReject(code) => {
                            if let Some(e) = esc.as_mut() {
                                e.flasher.reject(code);
                            }
                        }
                        Request::FlashProgress(offset, length) => {
                            if let Some(e) = esc.as_mut() {
                                e.flasher.record_chunk(offset, length, now);
                            }
                        }
                        Request::FlashEnd => {
                            if let Some(e) = esc.as_mut() {
                                e.flasher.end_for_hardware(now, e.telemetry.esc_hardware_id);
                            }
                        }
                        Request::FlashAbort => {
                            if let Some(e) = esc.as_mut() {
                                e.flasher.abort();
                            }
                        }
                    }
                }
                let output = {
                    let mut s = shared.lock().unwrap();
                    let maintenance = s.ota
                        || s.maintenance_pending
                        || esc.as_ref().map(|e| e.active()).unwrap_or(false);
                    let o = controller.tick(Input {
                        now_ms: now,
                        raw,
                        button_down: unsafe { platform::eppg_gpio_read(1) } == 0,
                        maintenance,
                    });
                    s.state = o.state;
                    #[cfg(feature = "qemu")]
                    qemu::set_state(o.state as u32, now);
                    o
                };
                if role == Role::Remote {
                    if output.control_due {
                        let _ = remote.try_send(wireless::RemoteControl {
                            captured_at: now,
                            raw,
                            state: output.desired_state,
                            mode: controller.mode,
                        });
                    }
                    if now.wrapping_sub(last_mirrored) > 500 {
                        let mut s = shared.lock().unwrap();
                        s.telemetry.esc_status = 0;
                        s.telemetry.bms_status = 0;
                    }
                }
                if let Some(e) = esc.as_mut() {
                    if output.send_throttle {
                        e.throttle(output.pwm);
                    }
                    for _ in 0..64 {
                        if let Some(frame) = platform::can_receive() {
                            e.receive(frame, unsafe { platform::eppg_micros() });
                        } else {
                            break;
                        }
                    }
                    e.tick(now, output.state);
                    if output.effects.contains(Effects::ARM) {
                        e.motor_tone(true);
                    }
                    if output.effects.contains(Effects::DISARM) {
                        e.motor_tone(false);
                    }
                    for _ in 0..64 {
                        let Some(frame) = e.transport.peek_frame() else {
                            break;
                        };
                        if platform::can_send(frame) {
                            e.transport.pop_frame();
                        } else {
                            break;
                        }
                    }
                    let mut s = shared.lock().unwrap();
                    copy_esc(&mut s.telemetry, &e.telemetry);
                    s.relay = e.active();
                    s.transfer_quiet = e.quiet(now);
                    s.esc_status = e.relay.status.encode();
                    s.esc_result = e.relay.result.clone();
                    s.esc_fw_status = e.flasher.status();
                }
                {
                    let mut s = shared.lock().unwrap();
                    s.state = output.state;
                    s.telemetry.device_state = output.state as u8;
                    s.settings.performance_mode = controller.mode;
                    s.settings.armed_time = controller.armed_minutes;
                    s.telemetry.pot_raw = raw;
                    s.telemetry.uptime_ms = now;
                    s.telemetry.packet_id = s
                        .telemetry
                        .packet_id
                        .wrapping_add(output.control_due as u32);
                    s.last_throttle = now;
                    s.failsafe = controller.failsafe;
                    s.control_age = controller.control_age(now).unwrap_or(u32::MAX);
                    if let Some((session, sequence)) = controller.accepted_control() {
                        s.accepted_session = session;
                        s.accepted_sequence = sequence;
                    }
                }
                if output.effects.0 != 0 {
                    if output.effects.contains(Effects::PAIRING) {
                        let _ = bluetooth.pair.try_send(());
                    }
                    if output.effects.contains(Effects::ZERO_ALTITUDE) {
                        let _ = zero_tx.try_send(());
                    }
                    if output.effects.contains(Effects::PERSIST) {
                        let s = shared.lock().unwrap().settings.clone();
                        let _ = persist_tx.try_send(s);
                    }
                    let _ = feedback.try_send(feedback::Feedback::Effects(output.effects));
                    let _ = ui_tx.try_send(output.effects);
                }
                if now.wrapping_sub(last_monitor) >= 100 {
                    last_monitor = now;
                    let s = shared.lock().unwrap().clone();
                    let d = monitor_data(&s);
                    if role == Role::Remote {
                        let f = &s.telemetry;
                        let masks = [
                            f.esc_warning_alerts,
                            f.esc_critical_alerts,
                            f.bms_warning_alerts,
                            f.bms_critical_alerts,
                        ];
                        for id in 0..41 {
                            let group = if id < 26 { 0 } else { 2 };
                            let bit = if id < 26 { id } else { id - 26 };
                            monitors.levels[id] = if masks[group + 1] & (1u64 << bit) != 0 {
                                controller_core::monitor::Level::CritHigh
                            } else if masks[group] & (1u64 << bit) != 0 {
                                controller_core::monitor::Level::WarnHigh
                            } else {
                                controller_core::monitor::Level::Ok
                            };
                        }
                    } else {
                        monitors.update(now, d);
                    }
                    let masks = monitors.masks();
                    let mut target = shared.lock().unwrap();
                    target.levels = monitors.levels;
                    target.telemetry.esc_warning_alerts = masks[0];
                    target.telemetry.esc_critical_alerts = masks[1];
                    target.telemetry.bms_warning_alerts = masks[2];
                    target.telemetry.bms_critical_alerts = masks[3];
                    let warnings = monitors.levels.iter().filter(|l| l.warning()).count();
                    let critical = monitors.levels.iter().any(|l| l.critical());
                    drop(target);
                    if !critical && warnings > old_warnings {
                        let _ = feedback.try_send(feedback::Feedback::Warning);
                    }
                    old_warnings = warnings;
                    if let Some(e) = esc.as_mut() {
                        e.status_light(
                            output.state,
                            masks[0] | masks[2] != 0,
                            masks[1] | masks[3] != 0,
                        );
                    }
                }
                if now.wrapping_sub(last_health) >= 250 {
                    last_health = now;
                    if initialized {
                        platform::can_health();
                    }
                }
                unsafe {
                    sys::esp_task_wdt_reset();
                }
                periodic(&mut last, 10);
            }
        });
    }
    configure_thread(b"watchdog\0", 5000, 3, Core::Core0);
    crate::spawn_task(move || {
        unsafe {
            sys::esp_task_wdt_add(std::ptr::null_mut());
        }
        loop {
            diagnostics.lock().unwrap().snapshot(&shared);
            unsafe {
                sys::esp_task_wdt_reset();
            }
            platform::sleep(100);
        }
    });
    unsafe {
        let running = sys::esp_ota_get_running_partition();
        let mut state = 0;
        if !running.is_null()
            && sys::esp_ota_get_state_partition(running, &mut state) == 0
            && state == sys::esp_ota_img_states_t_ESP_OTA_IMG_PENDING_VERIFY
        {
            sys::esp_ota_mark_app_valid_cancel_rollback();
        }
    }
    loop {
        platform::sleep(1000);
    }
}
fn monitor_data(s: &Snapshot) -> Data {
    let f = &s.telemetry;
    Data {
        esc_connected: f.esc_status != 0,
        bms_connected: f.bms_status != 0,
        armed: s.state != State::Disarmed,
        esc_initialized: s.esc_initialized,
        bms_initialized: s.bms_initialized,
        baro_initialized: s.baro_initialized,
        esc_temps: [
            f.esc_temp_mos_dC as f32 / 10.,
            f.esc_temp_mcu_dC as f32 / 10.,
            f.esc_temp_cap_dC as f32 / 10.,
            if f.esc_temp_motor_dC == i16::MIN {
                f32::NAN
            } else {
                f.esc_temp_motor_dC as f32 / 10.
            },
        ],
        running_errors: f.esc_error,
        selfcheck_errors: f.esc_selfcheck,
        bms_temps: s.bms_temperatures,
        pack_voltage: f.bms_volts_dV as f32 / 10.,
        max_cell: f.bms_cell_max_mV as f32 / 1000.,
        min_cell: f.bms_cell_min_mV as f32 / 1000.,
        soc: f.bms_soc as f32,
        voltage_diff: f.bms_voltage_diff_mV as f32 / 1000.,
        charge_mos: f.bms_is_charge_mos != 0,
        discharge_mos: f.bms_is_discharge_mos != 0,
        baro_temp: s.baro_temperature,
        cpu_temp: s.cpu_temperature,
    }
}

fn copy_esc(to: &mut FastLink, from: &FastLink) {
    to.esc_status = from.esc_status.clone();
    to.esc_volts_dV = from.esc_volts_dV.clone();
    to.esc_amps_dA = from.esc_amps_dA.clone();
    to.esc_phase_current_dA = from.esc_phase_current_dA.clone();
    to.esc_rpm = from.esc_rpm.clone();
    to.esc_temp_mos_dC = from.esc_temp_mos_dC.clone();
    to.esc_temp_cap_dC = from.esc_temp_cap_dC.clone();
    to.esc_temp_mcu_dC = from.esc_temp_mcu_dC.clone();
    to.esc_temp_motor_dC = from.esc_temp_motor_dC.clone();
    to.esc_inPWM = from.esc_inPWM.clone();
    to.esc_outPWM = from.esc_outPWM.clone();
    to.esc_v_modulation = from.esc_v_modulation.clone();
    to.esc_error = from.esc_error.clone();
    to.esc_selfcheck = from.esc_selfcheck.clone();
    to.esc_hardware_id = from.esc_hardware_id.clone();
    to.esc_fw_version = from.esc_fw_version.clone();
    to.esc_bootloader_version = from.esc_bootloader_version.clone();
    to.esc_runtime_ms = from.esc_runtime_ms.clone();
    to.esc_sn_code = from.esc_sn_code.clone();
    to.esc_warning_alerts = from.esc_warning_alerts.clone();
    to.esc_critical_alerts = from.esc_critical_alerts.clone();
}

fn copy_bms(to: &mut FastLink, from: &FastLink) {
    to.bms_status = from.bms_status.clone();
    to.bms_soc = from.bms_soc.clone();
    to.bms_volts_dV = from.bms_volts_dV.clone();
    to.bms_amps_dA = from.bms_amps_dA.clone();
    to.bms_energy_cycle_mAh = from.bms_energy_cycle_mAh.clone();
    to.bms_battery_cycle = from.bms_battery_cycle.clone();
    to.bms_fail_level = from.bms_fail_level.clone();
    to.bms_is_charging = from.bms_is_charging.clone();
    to.bms_is_charge_mos = from.bms_is_charge_mos.clone();
    to.bms_is_discharge_mos = from.bms_is_discharge_mos.clone();
    to.bms_charge_wire = from.bms_charge_wire.clone();
    to.bms_low_soc_warning = from.bms_low_soc_warning.clone();
    to.bms_battery_ready = from.bms_battery_ready.clone();
    to.bms_highest_temp_C = from.bms_highest_temp_C.clone();
    to.bms_lowest_temp_C = from.bms_lowest_temp_C.clone();
    to.bms_cell_max_mV = from.bms_cell_max_mV.clone();
    to.bms_cell_min_mV = from.bms_cell_min_mV.clone();
    to.bms_voltage_diff_mV = from.bms_voltage_diff_mV.clone();
    to.bms_battery_id = from.bms_battery_id.clone();
    to.bms_type = from.bms_type.clone();
    to.bms_cell_voltages_mV = from.bms_cell_voltages_mV.clone();
    to.bms_temp_sensors_C = from.bms_temp_sensors_C.clone();
    to.bms_warning_alerts = from.bms_warning_alerts.clone();
    to.bms_critical_alerts = from.bms_critical_alerts.clone();
}
