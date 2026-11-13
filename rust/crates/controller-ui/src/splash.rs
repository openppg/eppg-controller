unsafe extern "C" fn title_opacity(object: *mut c_void, value: i32) {
    lv_obj_set_style_opa(object.cast(), value as u8, 0);
}
impl Ui {
    pub fn show_splash(&mut self, settings: &Settings, now: u32) {
        unsafe {
            if !self.splash.is_null() {
                self.show_main();
            }
            NOW.store(now, Ordering::Relaxed);
            let dark = settings.theme == 1;
            self.splash = lv_obj_create(null_mut());
            lv_screen_load(self.splash);
            lv_obj_remove_flag(self.splash, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_bg_color(
                self.splash,
                if dark { LVGL_BLACK() } else { LVGL_WHITE() },
                0,
            );
            let fg = if dark { LVGL_WHITE() } else { LVGL_BLACK() };
            let title = lv_label_create(self.splash);
            text(title, "OpenPPG");
            lv_obj_set_style_text_font(title, addr_of!(lv_font_montserrat_28), 0);
            text_color(title, fg);
            lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);
            let mut animation: lv_anim_t = std::mem::zeroed();
            lv_anim_init(&mut animation);
            lv_anim_set_var(&mut animation, title.cast());
            lv_anim_set_values(&mut animation, 0, 255);
            lv_anim_set_duration(&mut animation, 500);
            lv_anim_set_exec_cb(&mut animation, Some(title_opacity));
            lv_anim_start(&animation);
            let version = lv_label_create(self.splash);
            text(
                version,
                &format!("v{}.{}", settings.version_major, settings.version_minor),
            );
            lv_obj_set_style_text_font(version, addr_of!(lv_font_montserrat_16), 0);
            text_color(version, fg);
            lv_obj_align(version, LV_ALIGN_CENTER, 0, 0);
            let time = lv_label_create(self.splash);
            text(
                time,
                &format!(
                    "{:02}:{:02}",
                    settings.armed_time / 60,
                    settings.armed_time % 60
                ),
            );
            lv_obj_set_style_text_font(time, addr_of!(lv_font_montserrat_16), 0);
            text_color(time, fg);
            lv_obj_align(time, LV_ALIGN_BOTTOM_MID, 0, -20);
            self.splash_started = Some(now);
        }
    }
    pub fn show_main(&mut self) {
        unsafe {
            lv_screen_load(self.w.main_screen);
            if !self.splash.is_null() {
                lv_obj_delete(self.splash);
                self.splash = null_mut();
            }
            self.splash_started = None;
        }
    }
}
