impl Ui {
    unsafe fn setup_alerts(&mut self) {
        if self.w.main_screen.is_null() {
            return;
        }
        let CIRCLE_SIZE: i32 = 18;
        if self.w.warning_counter_circle.is_null() {
            self.w.warning_counter_circle = lv_led_create(self.w.main_screen);
            lv_obj_set_size(self.w.warning_counter_circle, CIRCLE_SIZE, CIRCLE_SIZE);
            if !self.w.altitude_char_labels[0].is_null() {
                lv_obj_align_to(
                    self.w.warning_counter_circle,
                    self.w.altitude_char_labels[0],
                    LV_ALIGN_OUT_TOP_LEFT,
                    1,
                    -2,
                );
            } else {
                lv_obj_align(self.w.warning_counter_circle, LV_ALIGN_TOP_LEFT, 6, 75);
            }
            lv_led_set_color(self.w.warning_counter_circle, LVGL_ORANGE());
            lv_led_on(self.w.warning_counter_circle);
            lv_obj_set_style_bg_grad_dir(
                self.w.warning_counter_circle,
                LV_GRAD_DIR_NONE,
                LV_PART_MAIN,
            );
            lv_obj_set_style_shadow_width(self.w.warning_counter_circle, 0, LV_PART_MAIN);
            lv_obj_set_style_shadow_opa(self.w.warning_counter_circle, LV_OPA_TRANSP, LV_PART_MAIN);
            lv_obj_add_flag(self.w.warning_counter_circle, LV_OBJ_FLAG_HIDDEN);
            self.w.warning_counter_label = lv_label_create(self.w.warning_counter_circle);
            lv_obj_set_style_text_font(
                self.w.warning_counter_label,
                addr_of!(lv_font_montserrat_12),
                0,
            );
            lv_obj_set_style_text_color(self.w.warning_counter_label, LVGL_BLACK(), 0);
            lv_obj_align(self.w.warning_counter_label, LV_ALIGN_CENTER, 0, 0);
            lv_label_set_text(self.w.warning_counter_label, c"0".as_ptr());
        }
        if self.w.critical_counter_circle.is_null() {
            self.w.critical_counter_circle = lv_led_create(self.w.main_screen);
            lv_obj_set_size(self.w.critical_counter_circle, CIRCLE_SIZE, CIRCLE_SIZE);
            if !self.w.warning_counter_circle.is_null() {
                lv_obj_align_to(
                    self.w.critical_counter_circle,
                    self.w.warning_counter_circle,
                    LV_ALIGN_OUT_RIGHT_BOTTOM,
                    4,
                    0,
                );
            } else {
                lv_obj_align(self.w.critical_counter_circle, LV_ALIGN_TOP_LEFT, 35, 75);
            }
            lv_led_set_color(self.w.critical_counter_circle, LVGL_RED());
            lv_led_on(self.w.critical_counter_circle);
            lv_obj_set_style_bg_grad_dir(
                self.w.critical_counter_circle,
                LV_GRAD_DIR_NONE,
                LV_PART_MAIN,
            );
            lv_obj_set_style_shadow_width(self.w.critical_counter_circle, 0, LV_PART_MAIN);
            lv_obj_set_style_shadow_opa(
                self.w.critical_counter_circle,
                LV_OPA_TRANSP,
                LV_PART_MAIN,
            );
            lv_obj_add_flag(self.w.critical_counter_circle, LV_OBJ_FLAG_HIDDEN);
            self.w.critical_counter_label = lv_label_create(self.w.critical_counter_circle);
            lv_obj_set_style_text_font(
                self.w.critical_counter_label,
                addr_of!(lv_font_montserrat_12),
                0,
            );
            lv_obj_set_style_text_color(self.w.critical_counter_label, LVGL_WHITE(), 0);
            lv_obj_align(self.w.critical_counter_label, LV_ALIGN_CENTER, 0, 0);
            lv_label_set_text(self.w.critical_counter_label, c"0".as_ptr());
        }
        if self.w.alert_text_label.is_null() {
            self.w.alert_text_label = lv_label_create(self.w.main_screen);
            lv_obj_set_style_text_font(self.w.alert_text_label, addr_of!(lv_font_montserrat_14), 0);
            lv_obj_set_style_text_color(self.w.alert_text_label, LVGL_ORANGE(), 0);
            if !self.w.warning_counter_circle.is_null() {
                lv_obj_align_to(
                    self.w.alert_text_label,
                    self.w.warning_counter_circle,
                    LV_ALIGN_OUT_RIGHT_MID,
                    4,
                    0,
                );
            } else {
                lv_obj_align(self.w.alert_text_label, LV_ALIGN_TOP_LEFT, 60, 75);
            }
            lv_obj_add_flag(self.w.alert_text_label, LV_OBJ_FLAG_HIDDEN);
        }
        if self.w.critical_text_label.is_null() {
            self.w.critical_text_label = lv_label_create(self.w.main_screen);
            lv_obj_set_style_text_font(
                self.w.critical_text_label,
                addr_of!(lv_font_montserrat_18),
                0,
            );
            lv_obj_set_style_text_color(self.w.critical_text_label, lv_color_make(255, 0, 0), 0);
            if !self.w.altitude_char_labels[0].is_null() {
                lv_obj_set_pos(
                    self.w.critical_text_label,
                    lv_obj_get_x(self.w.altitude_char_labels[0]) + 25,
                    lv_obj_get_y(self.w.altitude_char_labels[0]),
                );
            } else {
                lv_obj_align(self.w.critical_text_label, LV_ALIGN_TOP_LEFT, 30, 100);
            }
            lv_obj_add_flag(self.w.critical_text_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

fn abbreviation(id: usize, level: Level) -> &'static str {
    match id {
        0 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "MC-FT-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "MC-FT-L"
            } else {
                "MC-F"
            }
        }
        1 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "MC-CT-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "MC-CT-L"
            } else {
                "MC-C"
            }
        }
        2 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "MC-PT-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "MC-PT-L"
            } else {
                "MC-P"
            }
        }
        3 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "MC-MT-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "MC-MT-L"
            } else {
                "MC-M"
            }
        }
        4 => "MC-RE-0",
        5 => "MC-RE-1",
        6 => "MC-RE-2",
        7 => "MC-RE-6",
        8 => "MC-RE-7",
        9 => "MC-RW-5",
        10 => "MC-SE-0",
        11 => "MC-SE-1",
        12 => "MC-SE-2",
        13 => "MC-SE-3",
        14 => "MC-SE-4",
        15 => "MC-SE-5",
        16 => "MC-SE-6",
        17 => "MC-SE-7",
        18 => "MC-SE-8",
        19 => "MC-SE-9",
        20 => "MC-SE-10",
        21 => "MC-SE-11",
        22 => "MC-SE-13",
        23 => "MC-SE-14",
        24 => "MC-CAN",
        25 => "MC-LINK",
        26 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "BC-F-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "BC-F-L"
            } else {
                "BC-F"
            }
        }
        27 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "BC-B-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "BC-B-L"
            } else {
                "BC-B"
            }
        }
        28 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "BC-T1-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "BC-T1-L"
            } else {
                "BC-T1"
            }
        }
        29 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "BC-T2-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "BC-T2-L"
            } else {
                "BC-T2"
            }
        }
        30 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "BC-T3-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "BC-T3-L"
            } else {
                "BC-T3"
            }
        }
        31 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "BC-T4-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "BC-T4-L"
            } else {
                "BC-T4"
            }
        }
        32 => "BC-CV-H",
        33 => {
            if matches!(level, Level::WarnHigh | Level::CritHigh) {
                "BC-CV-H"
            } else if matches!(level, Level::WarnLow | Level::CritLow) {
                "BC-CV-L"
            } else {
                "BC-CV"
            }
        }
        34 => "BC-SOC",
        35 => "BC-Vtot",
        36 => "BC-dV",
        37 => "BC-CHG",
        38 => "BC-DSG",
        39 => "BC-CAN",
        40 => "BC-LINK",
        41 => "BARO-T",
        42 => "BARO-I",
        43 => "CPU-T",
        _ => "UNK-ERR",
    }
}
