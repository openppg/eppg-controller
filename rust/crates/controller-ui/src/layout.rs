// Rust construction of the production 160x128 LVGL9 layout.
impl Ui {
    unsafe fn setup(&mut self, darkMode: bool) {
        if !self.w.main_screen.is_null() {
            self.w.critical_border = null_mut();
            lv_obj_delete(self.w.main_screen);
        }
        if self.styles_initialized {
            lv_style_reset(&mut self.style_warning);
            lv_style_reset(&mut self.style_critical);
        }
        self.w = Widgets::default();
        self.keepalive_points.clear();
        self.w.main_screen = lv_obj_create(null_mut());
        lv_obj_remove_flag(self.w.main_screen, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(
            self.w.main_screen,
            if darkMode { LVGL_BLACK() } else { LVGL_WHITE() },
            LV_PART_MAIN,
        );
        self.w.battery_bar = lv_bar_create(self.w.main_screen);
        lv_obj_set_size(self.w.battery_bar, 160, 37);
        lv_obj_set_pos(self.w.battery_bar, 0, 0);
        lv_bar_set_range(self.w.battery_bar, 0, 100);
        lv_obj_set_style_bg_color(
            self.w.battery_bar,
            if darkMode { LVGL_BLACK() } else { LVGL_WHITE() },
            LV_PART_MAIN,
        );
        lv_obj_set_style_radius(self.w.battery_bar, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(self.w.battery_bar, 0, LV_PART_INDICATOR);
        self.w.battery_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.battery_label, LV_ALIGN_TOP_MID, 0, 3);
        lv_obj_set_style_text_font(self.w.battery_label, addr_of!(lv_font_montserrat_28), 0);
        lv_obj_set_style_text_color(
            self.w.battery_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_text_align(self.w.battery_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(self.w.battery_label, c"".as_ptr());
        self.w.voltage_left_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.voltage_left_label, LV_ALIGN_TOP_LEFT, 3, 12);
        lv_obj_set_style_text_font(
            self.w.voltage_left_label,
            addr_of!(lv_font_montserrat_12),
            0,
        );
        lv_obj_set_style_text_color(
            self.w.voltage_left_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_label_set_text(self.w.voltage_left_label, c"".as_ptr());
        self.w.voltage_right_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.voltage_right_label, LV_ALIGN_TOP_RIGHT, -3, 12);
        lv_obj_set_style_text_font(
            self.w.voltage_right_label,
            addr_of!(lv_font_montserrat_12),
            0,
        );
        lv_obj_set_style_text_color(
            self.w.voltage_right_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_text_align(self.w.voltage_right_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_text(self.w.voltage_right_label, c"".as_ptr());
        let mut power_char_width: i32 = 16;
        let mut power_decimal_width: i32 = 8;
        let mut power_char_height: i32 = 28;
        let mut power_start_x: i32 = 0;
        let mut power_start_y: i32 = 54 - 14;
        let power_char_positions: [i32; 4] = [
            power_start_x,
            power_start_x + power_char_width,
            power_start_x + 2 * power_char_width,
            power_start_x + 2 * power_char_width + power_decimal_width,
        ];
        for i in 0usize..4 {
            self.w.power_char_labels[i] = lv_label_create(self.w.main_screen);
            lv_obj_set_style_text_font(
                self.w.power_char_labels[i],
                addr_of!(lv_font_montserrat_24),
                0,
            );
            lv_obj_set_style_text_color(
                self.w.power_char_labels[i],
                if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
                0,
            );
            lv_obj_set_style_text_align(self.w.power_char_labels[i], LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_pad_all(self.w.power_char_labels[i], 0, 0);
            let mut width: i32 = if i == 2 {
                power_decimal_width
            } else {
                power_char_width
            };
            lv_obj_set_size(self.w.power_char_labels[i], width, power_char_height);
            lv_obj_set_pos(
                self.w.power_char_labels[i],
                power_char_positions[i],
                power_start_y,
            );
            lv_label_set_text(self.w.power_char_labels[i], c"".as_ptr());
        }
        self.w.power_unit_label = lv_label_create(self.w.main_screen);
        lv_obj_align(
            self.w.power_unit_label,
            LV_ALIGN_LEFT_MID,
            power_start_x + 4 * power_char_width + power_decimal_width + 2 - 20 + 4,
            -7,
        );
        lv_obj_set_style_text_font(self.w.power_unit_label, addr_of!(lv_font_montserrat_10), 0);
        lv_obj_set_style_text_color(
            self.w.power_unit_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_label_set_text(self.w.power_unit_label, c"kW".as_ptr());
        self.w.perf_mode_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.perf_mode_label, LV_ALIGN_RIGHT_MID, -14, -17);
        lv_obj_set_style_text_font(self.w.perf_mode_label, addr_of!(lv_font_montserrat_12), 0);
        lv_obj_set_style_text_color(
            self.w.perf_mode_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_text_align(self.w.perf_mode_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_text(self.w.perf_mode_label, c"".as_ptr());
        self.w.armed_time_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.armed_time_label, LV_ALIGN_RIGHT_MID, -15, -3);
        lv_obj_set_style_text_font(self.w.armed_time_label, addr_of!(lv_font_montserrat_14), 0);
        lv_obj_set_style_text_color(
            self.w.armed_time_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_text_align(self.w.armed_time_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_text(self.w.armed_time_label, c"".as_ptr());
        let mut char_width: i32 = 19;
        let mut decimal_width: i32 = 8;
        let mut char_height: i32 = 30;
        let mut unit_width: i32 = 12;
        let mut total_width: i32 = 5 * char_width + decimal_width + unit_width;
        let mut start_x: i32 = 120 - total_width - 5;
        let mut start_y: i32 = 128 - 5 - char_height;
        let char_positions: [i32; 7] = [
            start_x,
            start_x + char_width,
            start_x + 2 * char_width,
            start_x + 3 * char_width,
            start_x + 4 * char_width,
            start_x + 4 * char_width + decimal_width,
            start_x + 5 * char_width + decimal_width,
        ];
        for i in 0usize..7 {
            self.w.altitude_char_labels[i] = lv_label_create(self.w.main_screen);
            if i == 6 {
                lv_obj_set_style_text_font(
                    self.w.altitude_char_labels[i],
                    addr_of!(lv_font_montserrat_12),
                    0,
                );
            } else {
                lv_obj_set_style_text_font(
                    self.w.altitude_char_labels[i],
                    addr_of!(lv_font_montserrat_28),
                    0,
                );
            }
            lv_obj_set_style_text_color(
                self.w.altitude_char_labels[i],
                if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
                0,
            );
            lv_obj_set_style_text_align(self.w.altitude_char_labels[i], LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_pad_all(self.w.altitude_char_labels[i], 0, 0);
            lv_obj_set_style_bg_opa(self.w.altitude_char_labels[i], LV_OPA_0, 0);
            lv_obj_set_style_border_width(self.w.altitude_char_labels[i], 0, 0);
            lv_obj_set_style_outline_width(self.w.altitude_char_labels[i], 0, 0);
            lv_obj_set_style_shadow_width(self.w.altitude_char_labels[i], 0, 0);
            lv_obj_set_style_border_opa(self.w.altitude_char_labels[i], LV_OPA_0, 0);
            lv_obj_set_style_outline_opa(self.w.altitude_char_labels[i], LV_OPA_0, 0);
            lv_obj_set_style_shadow_opa(self.w.altitude_char_labels[i], LV_OPA_0, 0);
            lv_obj_set_style_bg_color(self.w.altitude_char_labels[i], lv_color_hex(0x000000), 0);
            let mut width: i32 = char_width;
            if i == 4 {
                width = decimal_width;
            } else if i == 6 {
                width = unit_width;
            }
            lv_obj_set_size(self.w.altitude_char_labels[i], width, char_height);
            let mut y_pos: i32 = start_y;
            if i == 6 {
                y_pos += 11;
            }
            lv_obj_set_pos(self.w.altitude_char_labels[i], char_positions[i], y_pos);
            lv_label_set_text(self.w.altitude_char_labels[i], c"".as_ptr());
        }
        self.w.batt_temp_bg = self.createTempBackground(self.w.main_screen, 117, 70, 31, 19);
        self.w.esc_temp_bg = self.createTempBackground(self.w.main_screen, 117, 89, 31, 20);
        self.w.motor_temp_bg = self.createTempBackground(self.w.main_screen, 117, 109, 31, 19);
        self.w.batt_temp_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.batt_temp_label, LV_ALIGN_BOTTOM_RIGHT, -11, -39);
        lv_obj_set_style_text_font(self.w.batt_temp_label, addr_of!(lv_font_montserrat_12), 0);
        lv_obj_set_style_text_color(
            self.w.batt_temp_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_bg_opa(self.w.batt_temp_label, LV_OPA_0, 0);
        lv_obj_set_style_border_width(self.w.batt_temp_label, 0, 0);
        lv_obj_set_style_outline_width(self.w.batt_temp_label, 0, 0);
        lv_obj_set_style_shadow_width(self.w.batt_temp_label, 0, 0);
        lv_obj_set_style_bg_color(self.w.batt_temp_label, lv_color_hex(0x000000), 0);
        lv_obj_set_style_pad_left(self.w.batt_temp_label, 2, 0);
        lv_obj_set_style_pad_right(self.w.batt_temp_label, 2, 0);
        lv_obj_set_style_pad_top(self.w.batt_temp_label, 1, 0);
        lv_obj_set_style_pad_bottom(self.w.batt_temp_label, 1, 0);
        lv_obj_set_style_text_align(self.w.batt_temp_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_text(self.w.batt_temp_label, c"-".as_ptr());
        self.w.batt_letter_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.batt_letter_label, LV_ALIGN_BOTTOM_LEFT, 116, -41);
        lv_obj_set_style_text_font(self.w.batt_letter_label, addr_of!(lv_font_montserrat_10), 0);
        lv_obj_set_style_text_color(
            self.w.batt_letter_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_bg_opa(self.w.batt_letter_label, LV_OPA_0, 0);
        lv_obj_set_style_border_width(self.w.batt_letter_label, 0, 0);
        lv_obj_set_style_shadow_width(self.w.batt_letter_label, 0, 0);
        lv_obj_set_style_pad_left(self.w.batt_letter_label, 2, 0);
        lv_obj_set_style_pad_right(self.w.batt_letter_label, 2, 0);
        lv_obj_set_style_pad_top(self.w.batt_letter_label, 1, 0);
        lv_obj_set_style_pad_bottom(self.w.batt_letter_label, 1, 0);
        lv_label_set_text(self.w.batt_letter_label, c"B".as_ptr());
        self.w.esc_temp_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.esc_temp_label, LV_ALIGN_BOTTOM_RIGHT, -11, -20);
        lv_obj_set_style_text_font(self.w.esc_temp_label, addr_of!(lv_font_montserrat_12), 0);
        lv_obj_set_style_text_color(
            self.w.esc_temp_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_bg_opa(self.w.esc_temp_label, LV_OPA_0, 0);
        lv_obj_set_style_border_width(self.w.esc_temp_label, 0, 0);
        lv_obj_set_style_outline_width(self.w.esc_temp_label, 0, 0);
        lv_obj_set_style_shadow_width(self.w.esc_temp_label, 0, 0);
        lv_obj_set_style_bg_color(self.w.esc_temp_label, lv_color_hex(0x000000), 0);
        lv_obj_set_style_pad_left(self.w.esc_temp_label, 2, 0);
        lv_obj_set_style_pad_right(self.w.esc_temp_label, 2, 0);
        lv_obj_set_style_pad_top(self.w.esc_temp_label, 1, 0);
        lv_obj_set_style_pad_bottom(self.w.esc_temp_label, 1, 0);
        lv_obj_set_style_text_align(self.w.esc_temp_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_text(self.w.esc_temp_label, c"-".as_ptr());
        self.w.esc_letter_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.esc_letter_label, LV_ALIGN_BOTTOM_LEFT, 116, -22);
        lv_obj_set_style_text_font(self.w.esc_letter_label, addr_of!(lv_font_montserrat_10), 0);
        lv_obj_set_style_text_color(
            self.w.esc_letter_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_bg_opa(self.w.esc_letter_label, LV_OPA_0, 0);
        lv_obj_set_style_border_width(self.w.esc_letter_label, 0, 0);
        lv_obj_set_style_shadow_width(self.w.esc_letter_label, 0, 0);
        lv_obj_set_style_pad_left(self.w.esc_letter_label, 2, 0);
        lv_obj_set_style_pad_right(self.w.esc_letter_label, 2, 0);
        lv_obj_set_style_pad_top(self.w.esc_letter_label, 1, 0);
        lv_obj_set_style_pad_bottom(self.w.esc_letter_label, 1, 0);
        lv_label_set_text(self.w.esc_letter_label, c"E".as_ptr());
        self.w.motor_temp_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.motor_temp_label, LV_ALIGN_BOTTOM_RIGHT, -11, 0);
        lv_obj_set_style_text_font(self.w.motor_temp_label, addr_of!(lv_font_montserrat_12), 0);
        lv_obj_set_size(self.w.motor_temp_label, 25, 18);
        lv_obj_add_flag(self.w.motor_temp_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
        lv_obj_set_style_text_color(
            self.w.motor_temp_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_bg_opa(self.w.motor_temp_label, LV_OPA_0, 0);
        lv_obj_set_style_border_width(self.w.motor_temp_label, 0, 0);
        lv_obj_set_style_outline_width(self.w.motor_temp_label, 0, 0);
        lv_obj_set_style_shadow_width(self.w.motor_temp_label, 0, 0);
        lv_obj_set_style_bg_color(self.w.motor_temp_label, lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_opa(self.w.motor_temp_label, LV_OPA_0, 0);
        lv_obj_set_style_outline_opa(self.w.motor_temp_label, LV_OPA_0, 0);
        lv_obj_set_style_shadow_opa(self.w.motor_temp_label, LV_OPA_0, 0);
        lv_obj_set_style_pad_left(self.w.motor_temp_label, 2, 0);
        lv_obj_set_style_pad_right(self.w.motor_temp_label, 2, 0);
        lv_obj_set_style_pad_top(self.w.motor_temp_label, 1, 0);
        lv_obj_set_style_pad_bottom(self.w.motor_temp_label, 1, 0);
        lv_obj_set_style_text_align(self.w.motor_temp_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_text(self.w.motor_temp_label, c"-".as_ptr());
        self.w.motor_letter_label = lv_label_create(self.w.main_screen);
        lv_obj_align(self.w.motor_letter_label, LV_ALIGN_BOTTOM_LEFT, 116, -1);
        lv_obj_set_style_text_font(
            self.w.motor_letter_label,
            addr_of!(lv_font_montserrat_10),
            0,
        );
        lv_obj_set_size(self.w.motor_letter_label, 12, 15);
        lv_obj_add_flag(self.w.motor_letter_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
        lv_obj_set_style_text_color(
            self.w.motor_letter_label,
            if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
            0,
        );
        lv_obj_set_style_bg_opa(self.w.motor_letter_label, LV_OPA_0, 0);
        lv_obj_set_style_border_width(self.w.motor_letter_label, 0, 0);
        lv_obj_set_style_shadow_width(self.w.motor_letter_label, 0, 0);
        lv_obj_set_style_outline_width(self.w.motor_letter_label, 0, 0);
        lv_obj_set_style_border_opa(self.w.motor_letter_label, LV_OPA_0, 0);
        lv_obj_set_style_outline_opa(self.w.motor_letter_label, LV_OPA_0, 0);
        lv_obj_set_style_shadow_opa(self.w.motor_letter_label, LV_OPA_0, 0);
        lv_obj_set_style_bg_color(self.w.motor_letter_label, lv_color_hex(0x000000), 0);
        lv_obj_set_style_pad_left(self.w.motor_letter_label, 2, 0);
        lv_obj_set_style_pad_right(self.w.motor_letter_label, 2, 0);
        lv_obj_set_style_pad_top(self.w.motor_letter_label, 1, 0);
        lv_obj_set_style_pad_bottom(self.w.motor_letter_label, 1, 0);
        lv_label_set_text(self.w.motor_letter_label, c"M".as_ptr());
        let h_line1 = lv_line_create(self.w.main_screen);
        let h_line1_points = self.keep_points(&[(0, 37), (160, 37)]);
        lv_line_set_points(h_line1, h_line1_points, 2);
        lv_obj_set_style_line_color(h_line1, LVGL_GRAY(), LV_PART_MAIN);
        lv_obj_set_style_line_width(h_line1, 1, LV_PART_MAIN);
        let h_line2 = lv_line_create(self.w.main_screen);
        let h_line2_points = self.keep_points(&[(0, 70), (148, 70)]);
        lv_line_set_points(h_line2, h_line2_points, 2);
        lv_obj_set_style_line_color(h_line2, LVGL_GRAY(), LV_PART_MAIN);
        lv_obj_set_style_line_width(h_line2, 1, LV_PART_MAIN);
        let v_line1 = lv_line_create(self.w.main_screen);
        let v_line1_points = self.keep_points(&[(102, 37), (102, 70)]);
        lv_line_set_points(v_line1, v_line1_points, 2);
        lv_obj_set_style_line_color(v_line1, LVGL_GRAY(), LV_PART_MAIN);
        lv_obj_set_style_line_width(v_line1, 1, LV_PART_MAIN);
        let v_line2 = lv_line_create(self.w.main_screen);
        let v_line2_points = self.keep_points(&[(117, 70), (117, 128)]);
        lv_line_set_points(v_line2, v_line2_points, 2);
        lv_obj_set_style_line_color(v_line2, LVGL_GRAY(), LV_PART_MAIN);
        lv_obj_set_style_line_width(v_line2, 1, LV_PART_MAIN);
        let v_line3 = lv_line_create(self.w.main_screen);
        let v_line3_points = self.keep_points(&[(148, 37), (148, 128)]);
        lv_line_set_points(v_line3, v_line3_points, 2);
        lv_obj_set_style_line_color(v_line3, LVGL_GRAY(), LV_PART_MAIN);
        lv_obj_set_style_line_width(v_line3, 1, LV_PART_MAIN);
        let h_line3 = lv_line_create(self.w.main_screen);
        let h_line3_points = self.keep_points(&[(117, 89), (148, 89)]);
        lv_line_set_points(h_line3, h_line3_points, 2);
        lv_obj_set_style_line_color(h_line3, LVGL_GRAY(), LV_PART_MAIN);
        lv_obj_set_style_line_width(h_line3, 1, LV_PART_MAIN);
        let h_line4 = lv_line_create(self.w.main_screen);
        let h_line4_points = self.keep_points(&[(117, 109), (148, 109)]);
        lv_line_set_points(h_line4, h_line4_points, 2);
        lv_obj_set_style_line_color(h_line4, LVGL_GRAY(), LV_PART_MAIN);
        lv_obj_set_style_line_width(h_line4, 1, LV_PART_MAIN);
        self.w.arm_indicator = lv_obj_create(self.w.main_screen);
        lv_obj_set_size(self.w.arm_indicator, 46, 33);
        lv_obj_set_pos(self.w.arm_indicator, 102, 37);
        lv_obj_set_style_border_width(self.w.arm_indicator, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(self.w.arm_indicator, 0, LV_PART_MAIN);
        move_background(self.w.arm_indicator);
        lv_obj_add_flag(self.w.arm_indicator, LV_OBJ_FLAG_HIDDEN);
        self.w.cruise_icon_img = lv_image_create(self.w.main_screen);
        lv_image_set_src(
            self.w.cruise_icon_img,
            addr_of!(cruise_control_340255_30).cast(),
        );
        let icon_color = if darkMode {
            lv_color_white()
        } else {
            lv_color_black()
        };
        lv_obj_set_style_image_recolor(self.w.cruise_icon_img, icon_color, LV_PART_MAIN);
        lv_obj_set_style_image_recolor_opa(self.w.cruise_icon_img, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_align(self.w.cruise_icon_img, LV_ALIGN_CENTER, 5, -11);
        move_foreground(self.w.cruise_icon_img);
        lv_obj_add_flag(self.w.cruise_icon_img, LV_OBJ_FLAG_HIDDEN);
        self.w.charging_icon_img = lv_image_create(self.w.main_screen);
        lv_image_set_src(self.w.charging_icon_img, addr_of!(energy_539741_26).cast());
        lv_obj_align_to(
            self.w.charging_icon_img,
            self.w.battery_label,
            LV_ALIGN_OUT_RIGHT_MID,
            3,
            0,
        );
        lv_obj_set_style_image_recolor(self.w.charging_icon_img, icon_color, LV_PART_MAIN);
        lv_obj_set_style_image_recolor_opa(self.w.charging_icon_img, LV_OPA_COVER, LV_PART_MAIN);
        move_foreground(self.w.charging_icon_img);
        lv_obj_add_flag(self.w.charging_icon_img, LV_OBJ_FLAG_HIDDEN);
        self.w.arm_fail_warning_icon_img = lv_image_create(self.w.main_screen);
        lv_image_set_src(
            self.w.arm_fail_warning_icon_img,
            addr_of!(warning_2135850_30).cast(),
        );
        lv_obj_align(self.w.arm_fail_warning_icon_img, LV_ALIGN_CENTER, 12, -9);
        lv_obj_set_style_image_recolor(self.w.arm_fail_warning_icon_img, icon_color, LV_PART_MAIN);
        lv_obj_set_style_image_recolor_opa(
            self.w.arm_fail_warning_icon_img,
            LV_OPA_COVER,
            LV_PART_MAIN,
        );
        move_foreground(self.w.arm_fail_warning_icon_img);
        lv_obj_add_flag(self.w.arm_fail_warning_icon_img, LV_OBJ_FLAG_HIDDEN);
        self.w.ble_pairing_icon = lv_label_create(self.w.main_screen);
        lv_label_set_text(self.w.ble_pairing_icon, c"\u{f293}".as_ptr());
        lv_obj_set_style_text_font(self.w.ble_pairing_icon, addr_of!(lv_font_montserrat_14), 0);
        lv_obj_set_style_text_color(
            self.w.ble_pairing_icon,
            if darkMode {
                LVGL_DARK_BLE_BLUE()
            } else {
                LVGL_BLUE()
            },
            0,
        );
        lv_obj_set_pos(self.w.ble_pairing_icon, 103, 72);
        lv_obj_add_flag(self.w.ble_pairing_icon, LV_OBJ_FLAG_HIDDEN);
        let mut climb_section_start_y: i32 = 37;

        for i in 0usize..13 {
            let mut y_pos: i32 = climb_section_start_y + (i as i32 * 7);
            self.w.climb_rate_divider_lines[i] = lv_line_create(self.w.main_screen);
            self.climb_points[i][0].x = (148) as _;
            self.climb_points[i][0].y = (y_pos) as _;
            self.climb_points[i][1].x = (160) as _;
            self.climb_points[i][1].y = (y_pos) as _;
            lv_line_set_points(
                self.w.climb_rate_divider_lines[i],
                self.climb_points[i].as_ptr(),
                2,
            );
            if i == 6 {
                lv_obj_set_style_line_color(
                    self.w.climb_rate_divider_lines[i],
                    if darkMode { LVGL_WHITE() } else { LVGL_BLACK() },
                    LV_PART_MAIN,
                );
                lv_obj_set_style_line_width(self.w.climb_rate_divider_lines[i], 3, LV_PART_MAIN);
            } else {
                lv_obj_set_style_line_color(
                    self.w.climb_rate_divider_lines[i],
                    LVGL_GRAY(),
                    LV_PART_MAIN,
                );
                lv_obj_set_style_line_width(self.w.climb_rate_divider_lines[i], 1, LV_PART_MAIN);
            }
        }
        for i in 0usize..12 {
            let mut y_start: i32 = climb_section_start_y + (i as i32 * 7);
            let mut y_end: i32 = climb_section_start_y + ((i as i32 + 1) * 7);
            let mut section_height: i32 = y_end - y_start;
            self.w.climb_rate_fill_sections[i] = lv_obj_create(self.w.main_screen);
            lv_obj_set_size(self.w.climb_rate_fill_sections[i], 12, section_height);
            lv_obj_set_pos(self.w.climb_rate_fill_sections[i], 148, y_start);
            lv_obj_set_style_border_width(self.w.climb_rate_fill_sections[i], 0, LV_PART_MAIN);
            lv_obj_set_style_radius(self.w.climb_rate_fill_sections[i], 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(self.w.climb_rate_fill_sections[i], LV_OPA_0, LV_PART_MAIN);
            move_background(self.w.climb_rate_fill_sections[i]);
        }
        self.w.critical_border = lv_obj_create(self.w.main_screen);
        lv_obj_set_size(self.w.critical_border, 160, 128);
        lv_obj_set_pos(self.w.critical_border, 0, 0);
        lv_obj_remove_flag(self.w.critical_border, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_opa(self.w.critical_border, LV_OPA_0, LV_PART_MAIN);
        lv_obj_set_style_border_width(self.w.critical_border, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(self.w.critical_border, 0, LV_PART_MAIN);
        let edgeGeometry: [[i32; 4]; 4] = [
            [0, 0, 160, 4],
            [0, 128 - 4, 160, 4],
            [0, 4, 4, 128 - 8],
            [160 - 4, 4, 4, 128 - 8],
        ];
        for edge in edgeGeometry {
            let strip = lv_obj_create(self.w.critical_border);
            lv_obj_set_pos(strip, edge[0], edge[1]);
            lv_obj_set_size(strip, edge[2], edge[3]);
            lv_obj_remove_flag(strip, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_bg_color(strip, LVGL_RED(), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(strip, LV_OPA_0, LV_PART_MAIN);
            lv_obj_set_style_border_width(strip, 0, LV_PART_MAIN);
            lv_obj_set_style_radius(strip, 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(strip, 0, LV_PART_MAIN);
        }
        move_foreground(self.w.critical_border);
        self.setup_alerts();
        self.init_temp_styles();
        lv_screen_load(self.w.main_screen);
    }
}
