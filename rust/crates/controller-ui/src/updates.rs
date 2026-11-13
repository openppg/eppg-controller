impl Ui {
    pub fn set_monitors(&mut self, monitors: &Monitors) {
        self.alert_levels = monitors.levels;
        self.explicit_monitors = true;
    }
    pub fn update(&mut self, p: &FastLink, s: &Settings, pairing: bool, now: u32) {
        unsafe {
            NOW.store(now, Ordering::Relaxed);
            if let Some(start) = self.splash_started {
                if now.wrapping_sub(start) < 2000 {
                    self.render_at(now);
                    return;
                }
                self.show_main();
            }
            if self.theme != s.theme {
                self.theme = s.theme;
                self.setup(s.theme == 1);
                self.temperature_levels = [255; 3];
                self.last_voltage_mode = 255;
                self.charging_width = -1;
            }
            let dark = s.theme == 1;
            let fg = if dark { LVGL_WHITE() } else { LVGL_BLACK() };
            let bms = p.bms_status == 1;
            let esc = p.esc_status == 1;
            let armed = p.device_state != 0;
            if armed && !self.was_armed && !self.armed_start_explicit {
                self.armed_start = now;
            }
            self.was_armed = armed;
            if armed {
                self.last_armed = now;
            }
            let soc = self.battery_override.map_or(p.bms_soc as f32, |v| v.0);
            let volts = self.battery_override.map_or(
                if bms { p.bms_volts_dV } else { p.esc_volts_dV } as f32 / 10.,
                |v| v.1,
            );
            if bms {
                lv_bar_set_value(self.w.battery_bar, soc as i32, LV_ANIM_OFF);
                let c = if soc > 15. {
                    if dark {
                        color(0, 112, 48)
                    } else {
                        LVGL_GREEN()
                    }
                } else if soc >= 5. {
                    if dark {
                        color(128, 96, 0)
                    } else {
                        LVGL_YELLOW()
                    }
                } else if dark {
                    color(176, 0, 32)
                } else {
                    LVGL_RED()
                };
                bg(self.w.battery_bar, c, LV_PART_INDICATOR);
                text(self.w.battery_label, &format!("{}%", soc as i32));
                text_color(self.w.battery_label, fg);
            } else {
                lv_bar_set_value(self.w.battery_bar, 0, LV_ANIM_OFF);
                if esc {
                    text(self.w.battery_label, &format!("{volts:2.1}v"));
                    text_color(self.w.battery_label, fg);
                } else {
                    text(self.w.battery_label, "NO DATA");
                    text_color(self.w.battery_label, LVGL_RED());
                }
            }
            let voltage_mode = if bms {
                0
            } else if esc {
                1
            } else {
                2
            };
            if bms {
                text(
                    self.w.voltage_left_label,
                    &format!("{:2.2}v", p.bms_cell_min_mV as f32 / 1000.),
                );
                text_color(self.w.voltage_left_label, fg);
            } else if esc {
                text(self.w.voltage_left_label, "NO\nBMS");
                text_color(self.w.voltage_left_label, fg);
            } else {
                text(self.w.voltage_left_label, "");
            }
            if self.last_voltage_mode != voltage_mode {
                if voltage_mode < 2 {
                    lv_obj_align(
                        self.w.voltage_left_label,
                        LV_ALIGN_TOP_LEFT,
                        3,
                        if bms { 12 } else { 2 },
                    );
                }
                self.last_voltage_mode = voltage_mode;
            }
            if bms {
                text(self.w.voltage_right_label, &format!("{volts:2.0}v"));
            } else {
                text(self.w.voltage_right_label, "");
            }
            let mut power_text: [String; 4] = Default::default();
            if bms || esc {
                let amps = if bms { p.bms_amps_dA } else { p.esc_amps_dA } as f32 / 10.;
                let kw = self
                    .battery_override
                    .map_or(volts * amps / 1000., |v| v.2)
                    .max(0.);
                let total = (kw * 10. + 0.5) as i32;
                let whole = total / 10;
                if whole >= 10 {
                    power_text[0] = (whole / 10).to_string();
                }
                power_text[1] = (whole % 10).to_string();
                power_text[2] = ".".into();
                power_text[3] = (total % 10).to_string();
            }
            for (i, t) in power_text.iter().enumerate() {
                text(self.w.power_char_labels[i], t);
            }
            text(
                self.w.perf_mode_label,
                if s.performance_mode == 0 {
                    "CHILL "
                } else {
                    "SPORT"
                },
            );
            let secs = if self.armed_start > 0 {
                self.last_armed.wrapping_sub(self.armed_start) / 1000
            } else {
                0
            };
            text(
                self.w.armed_time_label,
                &format!("{:02}:{:02}", secs / 60, secs % 60),
            );
            let altitude = self.altitude_override.unwrap_or(if armed {
                p.altitude_cm as f32 / 100.
            } else {
                0.
            });
            let mut digits: [String; 7] = Default::default();
            if s.metric_alt {
                let display = if altitude > -0.05 && altitude < 0. {
                    0.
                } else {
                    altitude
                };
                let a = display.abs();
                let whole = a as i32;
                let decimal = ((a - whole as f32) * 10.).round() as i32;
                let mut left = 3;
                if whole >= 10 {
                    left = 2;
                }
                if whole >= 100 {
                    left = 1;
                }
                if whole >= 1000 {
                    left = 0;
                }
                if display < 0. && left > 0 {
                    digits[left - 1] = "-".into();
                }
                if whole / 1000 > 0 {
                    digits[0] = (whole / 1000).to_string();
                }
                if whole >= 100 {
                    digits[1] = ((whole / 100) % 10).to_string();
                }
                if whole >= 10 {
                    digits[2] = ((whole / 10) % 10).to_string();
                }
                digits[3] = (whole % 10).to_string();
                digits[4] = ".".into();
                digits[5] = decimal.to_string();
                digits[6] = "m".into();
                lv_obj_set_size(self.w.altitude_char_labels[4], 8, 30);
            } else {
                let feet_float = altitude * 3.28084;
                let feet = feet_float.abs().round() as i32;
                let mut left = 4;
                for (i, threshold) in [10000, 1000, 100, 10].into_iter().enumerate() {
                    if feet >= threshold {
                        left = left.min(i);
                    }
                }
                if feet_float < 0. && left > 0 {
                    digits[left - 1] = "-".into();
                }
                for (i, scale) in [10000, 1000, 100, 10, 1].into_iter().enumerate() {
                    if feet >= scale || scale == 1 {
                        digits[i] = ((feet / scale) % 10).to_string();
                    }
                }
                digits[6] = "f".into();
                lv_obj_set_size(self.w.altitude_char_labels[4], 17, 30);
            }
            for (i, d) in digits.iter().enumerate() {
                text(self.w.altitude_char_labels[i], d);
            }
            let battery = self.battery_temperature_override.unwrap_or_else(|| {
                p.bms_temp_sensors_C[2..6]
                    .iter()
                    .filter(|&&v| v != i8::MIN)
                    .max()
                    .map(|v| *v as f32)
            });
            let mos = if p.esc_temp_mos_dC == i16::MIN {
                None
            } else {
                Some(p.esc_temp_mos_dC as f32 / 10.)
            };
            let cap = if p.esc_temp_cap_dC == i16::MIN {
                None
            } else {
                Some(p.esc_temp_cap_dC as f32 / 10.)
            };
            let (esc_temp, warn, crit) = match (mos, cap) {
                (Some(m), Some(c)) if c > m => (Some(c), 85., 100.),
                (Some(m), _) => (Some(m), 90., 110.),
                (_, Some(c)) => (Some(c), 85., 100.),
                _ => (None, 90., 110.),
            };
            let motor = if p.esc_temp_motor_dC == i16::MIN {
                None
            } else {
                Some(p.esc_temp_motor_dC as f32 / 10.)
            };
            let tiles = [
                (
                    self.w.batt_temp_label,
                    self.w.batt_temp_bg,
                    if bms { battery } else { None },
                    50.,
                    56.,
                ),
                (
                    self.w.esc_temp_label,
                    self.w.esc_temp_bg,
                    if esc { esc_temp } else { None },
                    warn,
                    crit,
                ),
                (
                    self.w.motor_temp_label,
                    self.w.motor_temp_bg,
                    if esc { motor } else { None },
                    105.,
                    115.,
                ),
            ];
            for (i, (label, tile, temp, warn, crit)) in tiles.into_iter().enumerate() {
                let level = if let Some(t) = temp {
                    text(label, &(t as i32).to_string());
                    if t >= crit {
                        2
                    } else if t >= warn {
                        1
                    } else {
                        0
                    }
                } else {
                    text(label, "-");
                    0
                };
                if self.temperature_levels[i] != level {
                    lv_obj_remove_style(tile, &self.style_warning, 0);
                    lv_obj_remove_style(tile, &self.style_critical, 0);
                    if level != 0 {
                        let style = if level == 2 {
                            &mut self.style_critical
                        } else {
                            &mut self.style_warning
                        };
                        lv_obj_add_style(tile, style, 0);
                    }
                    visible(tile, level != 0);
                    self.temperature_levels[i] = level;
                }
            }
            bg(
                self.w.arm_indicator,
                if dark {
                    color(0, 96, 104)
                } else {
                    color(0, 255, 255)
                },
                0,
            );
            visible(self.w.arm_indicator, armed);
            let cruise_flashing = if let Some(start) = self.cruise_flash {
                if now.wrapping_sub(start) >= 1500 {
                    self.cruise_flash = None;
                    false
                } else {
                    visible(
                        self.w.cruise_icon_img,
                        now.wrapping_sub(start) / 250 % 2 == 0,
                    );
                    lv_obj_set_style_image_recolor(self.w.cruise_icon_img, LVGL_RED(), 0);
                    true
                }
            } else {
                false
            };
            // Match master: restore the theme foreground after flashing.
            if !cruise_flashing {
                visible(self.w.cruise_icon_img, p.device_state == 2);
                lv_obj_set_style_image_recolor(self.w.cruise_icon_img, fg, 0);
            }
            if let Some(start) = self.arm_fail_flash {
                if now.wrapping_sub(start) >= 1500 {
                    self.arm_fail_flash = None;
                    visible(self.w.arm_fail_warning_icon_img, false);
                } else {
                    visible(
                        self.w.arm_fail_warning_icon_img,
                        now.wrapping_sub(start) / 250 % 2 == 0,
                    );
                    lv_obj_set_style_image_recolor(self.w.arm_fail_warning_icon_img, LVGL_RED(), 0);
                }
            } else {
                visible(self.w.arm_fail_warning_icon_img, false);
            }
            let width = lv_obj_get_width(self.w.battery_label);
            if width != self.charging_width {
                lv_obj_align_to(
                    self.w.charging_icon_img,
                    self.w.battery_label,
                    LV_ALIGN_OUT_RIGHT_MID,
                    3,
                    0,
                );
                self.charging_width = width;
            }
            visible(self.w.charging_icon_img, bms && p.bms_is_charging != 0);
            if pairing {
                let start = *self.pairing_since.get_or_insert(now);
                visible(
                    self.w.ble_pairing_icon,
                    (now.wrapping_sub(start) / 500).is_multiple_of(2),
                );
            } else {
                self.pairing_since = None;
                visible(self.w.ble_pairing_icon, self.connected);
            }
            if now.wrapping_sub(self.last_altitude_at) > 200 {
                let rate = if self.last_altitude_at > 0 {
                    (altitude - self.last_altitude)
                        / (now.wrapping_sub(self.last_altitude_at) as f32 / 1000.)
                } else {
                    0.
                };
                self.vario(rate, dark);
                self.last_altitude = altitude;
                self.last_altitude_at = now;
            }
            self.update_alerts(p, now);
            self.render_at(now);
        }
    }
    unsafe fn vario(&mut self, rate: f32, dark: bool) {
        let positive = [
            color(0, 255, 0),
            color(255, 255, 0),
            color(255, 165, 0),
            color(255, 165, 0),
            color(255, 0, 255),
            color(255, 0, 255),
        ];
        let blue = if dark {
            color(74, 144, 226)
        } else {
            color(70, 130, 180)
        };
        let purple = if dark {
            color(192, 128, 255)
        } else {
            color(75, 0, 130)
        };
        let negative = [
            color(173, 216, 230),
            color(173, 216, 230),
            blue,
            blue,
            purple,
            purple,
        ];
        let count = (rate.abs() / 0.5) as usize;
        let count = count.min(6);
        for i in 0..12 {
            let active = if rate >= 0.25 {
                i < 6 && 5 - i < count
            } else if rate <= -0.25 {
                i >= 6 && i - 6 < count
            } else {
                false
            };
            if active {
                let value = if i < 6 {
                    positive[5 - i]
                } else {
                    negative[i - 6]
                };
                bg(self.w.climb_rate_fill_sections[i], value, 0);
            }
            let opa = if active { 255 } else { 0 };
            if lv_obj_get_style_bg_opa(self.w.climb_rate_fill_sections[i], 0) != opa {
                lv_obj_set_style_bg_opa(self.w.climb_rate_fill_sections[i], opa, 0);
            }
        }
    }
    unsafe fn altitude_visible(&mut self, show: bool) {
        for ptr in self.w.altitude_char_labels {
            visible(ptr, show);
        }
    }
    unsafe fn update_alerts(&mut self, p: &FastLink, now: u32) {
        if !self.explicit_monitors {
            self.alert_levels = [Level::Ok; 44];
            for (id, level) in self.alert_levels.iter_mut().enumerate().take(41) {
                let (bit, w, c) = if id < 26 {
                    (id, p.esc_warning_alerts, p.esc_critical_alerts)
                } else {
                    (id - 26, p.bms_warning_alerts, p.bms_critical_alerts)
                };
                *level = if c & (1u64 << bit) != 0 {
                    Level::CritHigh
                } else if w & (1u64 << bit) != 0 {
                    Level::WarnHigh
                } else {
                    Level::Ok
                };
            }
        }
        let warns: Vec<_> = self
            .alert_levels
            .iter()
            .enumerate()
            .filter_map(|(i, l)| l.warning().then_some(i))
            .collect();
        let crits: Vec<_> = self
            .alert_levels
            .iter()
            .enumerate()
            .filter_map(|(i, l)| l.critical().then_some(i))
            .collect();
        let changed = warns != self.warn_ids || crits != self.crit_ids;
        if changed {
            if warns != self.warn_ids {
                self.warn_idx = 0;
            }
            if crits != self.crit_ids {
                self.crit_idx = 0;
            }
            self.warn_ids = warns;
            self.crit_ids = crits;
            self.alert_rotation_at = now;
        } else if now.wrapping_sub(self.alert_rotation_at) >= 2000 {
            if !self.warn_ids.is_empty() {
                self.warn_idx = (self.warn_idx + 1) % self.warn_ids.len();
            }
            if !self.crit_ids.is_empty() {
                self.crit_idx = (self.crit_idx + 1) % self.crit_ids.len();
            }
            self.alert_rotation_at = now;
        }
        let warnings = self.warn_ids.len();
        let criticals = self.crit_ids.len();
        for (circle, label, count) in [
            (
                self.w.warning_counter_circle,
                self.w.warning_counter_label,
                warnings,
            ),
            (
                self.w.critical_counter_circle,
                self.w.critical_counter_label,
                criticals,
            ),
        ] {
            visible(circle, count > 0);
            if count > 0 {
                text(
                    label,
                    &if count > 9 {
                        "9+".to_string()
                    } else {
                        count.to_string()
                    },
                );
            }
        }
        visible(self.w.critical_text_label, criticals > 0);
        visible(self.w.alert_text_label, warnings > 0);
        self.altitude_visible(!(warnings > 0 && criticals > 0));
        if criticals > 0 {
            let id = self.crit_ids[self.crit_idx];
            text(
                self.w.critical_text_label,
                self.critical_label_override
                    .as_deref()
                    .unwrap_or(abbreviation(id, self.alert_levels[id])),
            );
            lv_obj_set_style_text_font(
                self.w.critical_text_label,
                addr_of!(lv_font_montserrat_18),
                0,
            );
            text_color(self.w.critical_text_label, LVGL_RED());
            lv_obj_align_to(
                self.w.critical_counter_circle,
                self.w.altitude_char_labels[0],
                LV_ALIGN_OUT_TOP_LEFT,
                1,
                -2,
            );
            lv_obj_align_to(
                self.w.critical_text_label,
                self.w.critical_counter_circle,
                LV_ALIGN_OUT_RIGHT_MID,
                4,
                0,
            );
        }
        if warnings > 0 {
            let id = self.warn_ids[self.warn_idx];
            text(
                self.w.alert_text_label,
                self.warning_label_override
                    .as_deref()
                    .unwrap_or(abbreviation(id, self.alert_levels[id])),
            );
            lv_obj_set_style_text_font(self.w.alert_text_label, addr_of!(lv_font_montserrat_18), 0);
            text_color(self.w.alert_text_label, color(200, 100, 0));
            if criticals > 0 {
                lv_obj_set_pos(
                    self.w.warning_counter_circle,
                    lv_obj_get_x(self.w.critical_counter_circle),
                    lv_obj_get_y(self.w.altitude_char_labels[0]) + 2,
                );
            } else {
                lv_obj_align_to(
                    self.w.warning_counter_circle,
                    self.w.altitude_char_labels[0],
                    LV_ALIGN_OUT_TOP_LEFT,
                    1,
                    -2,
                );
            }
            lv_obj_align_to(
                self.w.alert_text_label,
                self.w.warning_counter_circle,
                LV_ALIGN_OUT_RIGHT_MID,
                4,
                0,
            );
        }
        let on = if criticals > 0 {
            let start = *self.critical_since.get_or_insert(now);
            now.wrapping_sub(start) % 1000 < 300
        } else {
            self.critical_since = None;
            false
        };
        self.vibration_due = on && !self.last_border_on;
        self.last_border_on = on;
        for i in 0..lv_obj_get_child_count(self.w.critical_border) {
            let child = lv_obj_get_child(self.w.critical_border, i as i32);
            let opa = if on { 255 } else { 0 };
            if lv_obj_get_style_bg_opa(child, 0) != opa {
                lv_obj_set_style_bg_opa(child, opa, 0);
            }
        }
    }
}
