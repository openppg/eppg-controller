use controller_core::{
    monitor::{Level, Monitors},
    settings::Settings,
};
use controller_protocols::fastlink::FastLink;
use controller_ui::Ui;
use std::path::Path;
fn compare(name: &str, pixels: &[u16]) {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../../test/test_screenshots/reference")
        .join(format!("{name}.bmp"));
    let bytes =
        std::fs::read(&path).unwrap_or_else(|e| panic!("Missing golden {}: {e}", path.display()));
    let mut differences = 0;
    for y in 0..128 {
        for x in 0..160 {
            let offset = 54 + ((127 - y) * 160 + x) * 3;
            let px = pixels[y * 160 + x];
            let rgb = [
                (px & 31) as u32 * 255 / 31,
                ((px >> 5) & 63) as u32 * 255 / 63,
                ((px >> 11) & 31) as u32 * 255 / 31,
            ];
            if rgb[0] as u8 != bytes[offset]
                || rgb[1] as u8 != bytes[offset + 1]
                || rgb[2] as u8 != bytes[offset + 2]
            {
                differences += 1;
            }
        }
    }
    if differences > 0 {
        let output = std::env::temp_dir().join(format!("rust-{name}.bmp"));
        let mut result = bytes;
        for y in 0..128 {
            for x in 0..160 {
                let offset = 54 + ((127 - y) * 160 + x) * 3;
                let px = pixels[y * 160 + x];
                result[offset] = ((px & 31) as u32 * 255 / 31) as u8;
                result[offset + 1] = (((px >> 5) & 63) as u32 * 255 / 63) as u8;
                result[offset + 2] = (((px >> 11) & 31) as u32 * 255 / 31) as u8;
            }
        }
        std::fs::write(&output, result).unwrap();
        eprintln!(
            "{name}: {differences} differing pixels -> {}",
            output.display()
        );
    }
    assert_eq!(differences, 0, "Screenshot {name}");
}
fn packet() -> FastLink {
    FastLink {
        esc_status: 1,
        esc_volts_dV: 885,
        esc_amps_dA: 452,
        esc_temp_mos_dC: 520,
        esc_temp_cap_dC: 480,
        esc_temp_mcu_dC: 420,
        esc_temp_motor_dC: 650,
        esc_rpm: 12000,
        bms_status: 1,
        bms_soc: 72,
        bms_volts_dV: 885,
        bms_amps_dA: 452,
        bms_cell_max_mV: 3950,
        bms_cell_min_mV: 3900,
        bms_temp_sensors_C: [32, 30, 35, 33, i8::MIN, i8::MIN, i8::MIN, i8::MIN],
        ..FastLink::default()
    }
}
#[test]
fn production_screens_match_cpp_goldens() {
    let mut failures = vec![];
    for dark in [false, true] {
        for case in [
            "idle",
            "armed_flying",
            "armed_cruising",
            "low_battery",
            "high_altitude",
            "max_climb",
            "max_descent",
            "high_power",
            "charging",
            "esc_disconnected",
            "full_battery",
            "temp_warnings",
            "temp_critical",
            "ble_pairing",
            "warning_alerts",
            "critical_alerts",
            "splash",
        ] {
            let result = std::panic::catch_unwind(|| {
                let mut settings = Settings {
                    theme: dark as u8,
                    performance_mode: 1,
                    screen_rotation: 1,
                    armed_time: 125,
                    ..Settings::default()
                };
                let mut p = packet();
                let (mut soc, mut volts, mut power, mut altitude, mut elapsed, mut climb) =
                    (72., 88.5, 0., 0., 0, None);
                let mut pairing = false;
                match case {
                    "armed_flying" | "max_climb" | "max_descent" => {
                        soc = 65.;
                        volts = 86.;
                        power = 8.5;
                        altitude = 450.3;
                        elapsed = 180000;
                        p.device_state = 1;
                        climb = if case == "max_climb" {
                            Some(3.)
                        } else if case == "max_descent" {
                            Some(-3.)
                        } else {
                            None
                        };
                    }
                    "armed_cruising" => {
                        soc = 58.;
                        volts = 84.;
                        power = 6.2;
                        altitude = 820.7;
                        elapsed = 600000;
                        p.device_state = 2;
                    }
                    "low_battery" => {
                        soc = 8.;
                        volts = 72.;
                        power = 2.;
                        altitude = 200.;
                        elapsed = 300000;
                        p.device_state = 1;
                        p.bms_cell_min_mV = 3150;
                    }
                    "high_altitude" => {
                        soc = 45.;
                        volts = 82.;
                        power = 4.5;
                        altitude = 2456.8;
                        elapsed = 900000;
                        p.device_state = 1;
                    }
                    "high_power" => {
                        soc = 50.;
                        volts = 85.;
                        power = 18.5;
                        altitude = 300.;
                        elapsed = 60000;
                        p.device_state = 1;
                        p.esc_temp_mos_dC = 850;
                        p.esc_temp_motor_dC = 950;
                        settings.performance_mode = 2;
                    }
                    "charging" => {
                        soc = 85.;
                        volts = 96.;
                        power = -0.5;
                        p.esc_status = 0;
                        p.bms_is_charging = 1;
                    }
                    "esc_disconnected" => {
                        p.esc_status = 0;
                    }
                    "full_battery" => {
                        soc = 100.;
                        volts = 100.8;
                        p.bms_cell_min_mV = 4180;
                    }
                    "temp_warnings" => {
                        soc = 60.;
                        volts = 85.;
                        power = 6.;
                        altitude = 300.;
                        elapsed = 120000;
                        p.device_state = 1;
                        p.esc_temp_mos_dC = 950;
                        p.esc_temp_motor_dC = 1080;
                        p.bms_temp_sensors_C[2] = 52;
                    }
                    "temp_critical" => {
                        soc = 40.;
                        volts = 80.;
                        power = 12.;
                        altitude = 250.;
                        elapsed = 180000;
                        p.device_state = 1;
                        p.esc_temp_mos_dC = 1150;
                        p.esc_temp_motor_dC = 1200;
                        p.bms_temp_sensors_C[2] = 58;
                    }
                    "ble_pairing" => {
                        pairing = true;
                    }
                    _ => {}
                }
                if case == "warning_alerts" {
                    soc = 55.;
                    volts = 84.;
                    power = 5.;
                    altitude = 400.;
                    elapsed = 60000;
                    p.device_state = 1;
                    p.esc_temp_mos_dC = 920;
                }
                if case == "critical_alerts" {
                    soc = 35.;
                    volts = 78.;
                    power = 14.;
                    altitude = 200.;
                    elapsed = 300000;
                    p.device_state = 1;
                    p.esc_temp_mos_dC = 1150;
                    p.esc_temp_motor_dC = 1200;
                    p.bms_temp_sensors_C[2] = 58;
                }
                let mut ui = Ui::with_settings(&settings);
                if case == "splash" {
                    ui.show_splash(&settings, 0);
                    ui.render_at(500);
                    compare(
                        &format!("splash_{}", if dark { "dark" } else { "light" }),
                        ui.framebuffer(),
                    );
                    return;
                }
                // The legacy golden harness stubs alert abbreviations (E.MOS/MOT), unlike
                // production MC-FT/MC-MT. Supply the same display input for pixel parity;
                // production abbreviation strings have separate assertions in the UI unit tests.
                if case == "warning_alerts" {
                    ui.set_alert_labels(Some("E.MOS Hi"), None);
                    let mut m = Monitors::default();
                    m.levels[0] = Level::WarnHigh;
                    m.levels[3] = Level::WarnHigh;
                    ui.set_monitors(&m);
                }
                if case == "critical_alerts" {
                    ui.set_alert_labels(Some("E.MOS Hi"), Some("MOT HI"));
                    let mut m = Monitors::default();
                    m.levels[0] = Level::WarnHigh;
                    for id in [3, 4, 5] {
                        m.levels[id] = Level::CritHigh;
                    }
                    ui.set_monitors(&m);
                }
                ui.set_battery_readings(soc, volts, power);
                ui.set_altitude_reading(altitude);
                if elapsed != 0 {
                    ui.set_armed_start(1_000_000 - elapsed);
                }
                ui.update(&p, &settings, pairing, 1_000_000);
                if let Some(rate) = climb {
                    ui.set_vario(rate);
                    ui.render_at(1_000_000);
                }
                compare(
                    &format!("main_{case}_{}", if dark { "dark" } else { "light" }),
                    ui.framebuffer(),
                );
            });
            if result.is_err() {
                failures.push((dark, case));
            }
        }
    }
    assert!(failures.is_empty(), "Golden failures {failures:?}");
}
