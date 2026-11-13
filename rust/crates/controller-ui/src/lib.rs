//! Production LVGL9 interface shared by ESP32-S3 firmware and native framebuffer tests.
//! LVGL and every object belong to one UI task; Ui deliberately cannot be Send/Sync.
#![allow(non_snake_case, non_upper_case_globals, unused_mut)]
use controller_core::{
    monitor::{Level, Monitors},
    settings::Settings,
};
use controller_protocols::fastlink::FastLink;
use std::{
    ffi::{c_void, CStr, CString},
    marker::PhantomData,
    ptr::{addr_of, null_mut},
    rc::Rc,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
#[allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    improper_ctypes,
    unused_imports,
    unnecessary_transmutes
)]
#[allow(clippy::all)]
mod sys {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
use sys::*;
include!("widgets.rs");
const LV_ANIM_OFF: bool = false;
const LV_OPA_TRANSP: u8 = 0;
const LV_OPA_0: u8 = 0;
const LV_OPA_COVER: u8 = 255;
static LIVE: AtomicBool = AtomicBool::new(false);
static INITIALIZED: AtomicBool = AtomicBool::new(false);
static NOW: AtomicU32 = AtomicU32::new(0);
unsafe extern "C" fn tick() -> u32 {
    NOW.load(Ordering::Relaxed)
}
fn color(r: u8, g: u8, b: u8) -> lv_color_t {
    lv_color_t {
        red: r,
        green: g,
        blue: b,
    }
}
fn LVGL_BLACK() -> lv_color_t {
    color(0, 0, 0)
}
fn LVGL_WHITE() -> lv_color_t {
    color(255, 255, 255)
}
fn LVGL_GREEN() -> lv_color_t {
    color(0, 255, 0)
}
fn LVGL_YELLOW() -> lv_color_t {
    color(255, 255, 0)
}
fn LVGL_RED() -> lv_color_t {
    color(255, 0, 0)
}
fn LVGL_BLUE() -> lv_color_t {
    color(0, 0, 255)
}
fn LVGL_ORANGE() -> lv_color_t {
    color(255, 165, 0)
}
fn LVGL_GRAY() -> lv_color_t {
    color(128, 128, 128)
}
fn LVGL_DARK_BLE_BLUE() -> lv_color_t {
    color(64, 160, 255)
}
unsafe fn move_background(obj: *mut lv_obj_t) {
    lv_obj_move_to_index(obj, 0)
}
unsafe fn move_foreground(obj: *mut lv_obj_t) {
    lv_obj_move_to_index(obj, -1)
}
unsafe fn visible(obj: *mut lv_obj_t, show: bool) {
    if show {
        lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN)
    } else {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN)
    }
}
unsafe fn text(obj: *mut lv_obj_t, value: &str) {
    if CStr::from_ptr(lv_label_get_text(obj)).to_bytes() != value.as_bytes() {
        let c = CString::new(value).expect("UI text has no NUL");
        lv_label_set_text(obj, c.as_ptr());
    }
}
unsafe fn text_color(obj: *mut lv_obj_t, value: lv_color_t) {
    let prev = lv_obj_get_style_text_color(obj, 0);
    if prev.red != value.red || prev.green != value.green || prev.blue != value.blue {
        lv_obj_set_style_text_color(obj, value, 0);
    }
}
unsafe fn bg(obj: *mut lv_obj_t, value: lv_color_t, part: u32) {
    let prev = lv_obj_get_style_bg_color(obj, part);
    if prev.red != value.red || prev.green != value.green || prev.blue != value.blue {
        lv_obj_set_style_bg_color(obj, value, part);
    }
}
/// Optional native-panel transport. The slice is RGB565 in native byte order.
pub type PanelFlush = fn(x1: i32, y1: i32, x2: i32, y2: i32, pixels: &[u8]);
struct Sink {
    frame: Vec<u16>,
    panel: Option<PanelFlush>,
}
unsafe extern "C" fn flush(display: *mut lv_display_t, area: *const lv_area_t, pixels: *mut u8) {
    let sink = &mut *(lv_display_get_user_data(display) as *mut Sink);
    let a = &*area;
    let width = (a.x2 - a.x1 + 1) as usize;
    let height = (a.y2 - a.y1 + 1) as usize;
    let bytes = std::slice::from_raw_parts(pixels, width * height * 2);
    if !sink.frame.is_empty() {
        for y in 0..height {
            for x in 0..width {
                let off = (y * width + x) * 2;
                let dx = a.x1 + x as i32;
                let dy = a.y1 + y as i32;
                if (0..160).contains(&dx) && (0..128).contains(&dy) {
                    sink.frame[dy as usize * 160 + dx as usize] =
                        u16::from_ne_bytes([bytes[off], bytes[off + 1]]);
                }
            }
        }
    }
    if let Some(panel) = sink.panel {
        panel(a.x1, a.y1, a.x2, a.y2, bytes)
    }
    lv_display_flush_ready(display);
}
/// Single-owner screen, with Rust-owned widgets, geometry, animation state and buffers.
pub struct Ui {
    w: Widgets,
    display: *mut lv_display_t,
    sink: Box<Sink>,
    _buffers: [Box<[u8]>; 1],
    keepalive_points: Vec<Box<[lv_point_precise_t]>>,
    climb_points: Box<[[lv_point_precise_t; 2]; 13]>,
    // LVGL retains these addresses; moving Ui must not move its styles.
    style_warning: Box<lv_style_t>,
    style_critical: Box<lv_style_t>,
    styles_initialized: bool,
    splash: *mut lv_obj_t,
    splash_started: Option<u32>,
    theme: u8,
    rotation: u8,
    battery_override: Option<(f32, f32, f32)>,
    battery_temperature_override: Option<Option<f32>>,
    altitude_override: Option<f32>,
    armed_start_explicit: bool,
    armed_start: u32,
    last_armed: u32,
    was_armed: bool,
    last_altitude: f32,
    last_altitude_at: u32,
    cruise_flash: Option<u32>,
    arm_fail_flash: Option<u32>,
    critical_since: Option<u32>,
    pairing_since: Option<u32>,
    alert_levels: [Level; 44],
    explicit_monitors: bool,
    warn_ids: Vec<usize>,
    crit_ids: Vec<usize>,
    warn_idx: usize,
    crit_idx: usize,
    alert_rotation_at: u32,
    warning_label_override: Option<String>,
    critical_label_override: Option<String>,
    connected: bool,
    temperature_levels: [u8; 3],
    last_voltage_mode: u8,
    charging_width: i32,
    pub vibration_due: bool,
    last_border_on: bool,
    _single_owner: PhantomData<Rc<()>>,
}
impl Default for Ui {
    fn default() -> Self {
        Self::new()
    }
}
impl Ui {
    pub fn new() -> Self {
        Self::with_settings(&Settings::default())
    }
    pub fn with_settings(settings: &Settings) -> Self {
        unsafe {
            assert!(
                !LIVE.swap(true, Ordering::AcqRel),
                "LVGL supports one Ui owner"
            );
            if !INITIALIZED.swap(true, Ordering::AcqRel) {
                lv_init();
                lv_tick_set_cb(Some(tick));
            }
            let display = lv_display_create(160, 128);
            assert!(!display.is_null());
            let mut sink = Box::new(Sink {
                frame: if cfg!(target_os = "espidf") {
                    Vec::new()
                } else {
                    vec![0; 160 * 128]
                },
                panel: None,
            });
            // The panel flush is synchronous; a second buffer cannot overlap DMA.
            // Sixteen RGB565 rows leave internal RAM available to BLE and services.
            let mut buffers = [vec![0u8; 160 * 16 * 2].into_boxed_slice()];
            lv_display_set_user_data(display, (&mut *sink as *mut Sink).cast());
            lv_display_set_flush_cb(display, Some(flush));
            lv_display_set_buffers(
                display,
                buffers[0].as_mut_ptr().cast(),
                std::ptr::null_mut(),
                (160 * 16 * 2) as u32,
                LV_DISPLAY_RENDER_MODE_PARTIAL,
            );
            lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
            let theme = lv_theme_default_init(
                display,
                lv_palette_main(LV_PALETTE_BLUE),
                lv_palette_main(LV_PALETTE_AMBER),
                settings.theme == 1,
                addr_of!(lv_font_montserrat_14),
            );
            lv_display_set_theme(display, theme);
            let mut ui = Self {
                w: Widgets::default(),
                display,
                sink,
                _buffers: buffers,
                keepalive_points: Vec::new(),
                climb_points: Box::new(std::mem::zeroed()),
                style_warning: Box::new(std::mem::zeroed()),
                style_critical: Box::new(std::mem::zeroed()),
                styles_initialized: false,
                splash: null_mut(),
                splash_started: None,
                theme: settings.theme,
                rotation: settings.screen_rotation,
                battery_override: None,
                battery_temperature_override: None,
                altitude_override: None,
                armed_start_explicit: false,
                armed_start: 0,
                last_armed: 0,
                was_armed: false,
                last_altitude: 0.,
                last_altitude_at: 0,
                cruise_flash: None,
                arm_fail_flash: None,
                critical_since: None,
                pairing_since: None,
                alert_levels: [Level::Ok; 44],
                explicit_monitors: false,
                warn_ids: Vec::new(),
                crit_ids: Vec::new(),
                warn_idx: 0,
                crit_idx: 0,
                alert_rotation_at: 0,
                warning_label_override: None,
                critical_label_override: None,
                connected: false,
                temperature_levels: [255; 3],
                last_voltage_mode: 255,
                charging_width: -1,
                vibration_due: false,
                last_border_on: false,
                _single_owner: PhantomData,
            };
            ui.setup(settings.theme == 1);
            ui
        }
    }
    pub fn rotation(&self) -> u8 {
        self.rotation
    }
    pub fn set_panel_flush(&mut self, callback: PanelFlush) {
        self.sink.panel = Some(callback)
    }
    pub fn framebuffer(&self) -> &[u16] {
        &self.sink.frame
    }
    pub fn set_battery_readings(&mut self, soc: f32, volts: f32, power_kw: f32) {
        self.battery_override = Some((soc, volts, power_kw));
    }
    pub fn set_battery_temperature(&mut self, value: Option<f32>) {
        self.battery_temperature_override = Some(value.filter(|v| v.is_finite()));
    }
    pub fn clear_battery_readings(&mut self) {
        self.battery_override = None;
    }
    pub fn set_altitude_reading(&mut self, altitude: f32) {
        self.altitude_override = Some(altitude);
    }
    pub fn set_vario(&mut self, rate: f32) {
        unsafe {
            self.vario(rate, self.theme == 1);
        }
    }
    /// Supply externally aggregated display text; useful for integrations with their own labels.
    pub fn set_alert_labels(&mut self, warning: Option<&str>, critical: Option<&str>) {
        self.warning_label_override = warning.map(str::to_owned);
        self.critical_label_override = critical.map(str::to_owned);
    }
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected
    }
    pub fn set_armed_start(&mut self, start: u32) {
        self.armed_start = start;
        self.armed_start_explicit = true;
    }
    pub fn flash_cruise(&mut self, now: u32) {
        self.cruise_flash = Some(now)
    }
    pub fn flash_arm_failure(&mut self, now: u32) {
        self.arm_fail_flash = Some(now)
    }
    pub fn render_at(&mut self, now: u32) {
        NOW.store(now, Ordering::Relaxed);
        unsafe {
            lv_timer_handler();
            lv_refr_now(self.display);
        }
    }
    unsafe fn keep_points(&mut self, points: &[(i32, i32)]) -> *const lv_point_precise_t {
        let list: Box<[_]> = points
            .iter()
            .map(|&(x, y)| lv_point_precise_t {
                x: x as _,
                y: y as _,
            })
            .collect();
        let ptr = list.as_ptr();
        self.keepalive_points.push(list);
        ptr
    }
    unsafe fn createTempBackground(
        &mut self,
        parent: *mut lv_obj_t,
        x: i32,
        y: i32,
        width: i32,
        height: i32,
    ) -> *mut lv_obj_t {
        let obj = lv_obj_create(parent);
        lv_obj_set_size(obj, width, height);
        lv_obj_set_pos(obj, x, y);
        lv_obj_set_style_border_width(obj, 0, 0);
        lv_obj_set_style_radius(obj, 0, 0);
        lv_obj_set_style_bg_opa(obj, 255, 0);
        lv_obj_set_style_shadow_width(obj, 0, 0);
        lv_obj_set_style_outline_width(obj, 0, 0);
        lv_obj_set_style_pad_all(obj, 0, 0);
        lv_obj_set_style_border_opa(obj, 0, 0);
        lv_obj_set_style_outline_opa(obj, 0, 0);
        lv_obj_set_style_shadow_opa(obj, 0, 0);
        move_background(obj);
        visible(obj, false);
        obj
    }
    unsafe fn init_temp_styles(&mut self) {
        self.styles_initialized = true;
        lv_style_init(&mut *self.style_warning);
        lv_style_set_bg_color(&mut *self.style_warning, LVGL_YELLOW());
        lv_style_set_text_color(&mut *self.style_warning, LVGL_BLACK());
        lv_style_set_bg_opa(&mut *self.style_warning, 255);
        lv_style_init(&mut *self.style_critical);
        lv_style_set_bg_color(&mut *self.style_critical, LVGL_RED());
        lv_style_set_text_color(&mut *self.style_critical, LVGL_BLACK());
        lv_style_set_bg_opa(&mut *self.style_critical, 255);
    }
}
include!("layout.rs");
include!("splash.rs");
include!("alerts.rs");
include!("updates.rs");
impl Drop for Ui {
    fn drop(&mut self) {
        unsafe {
            lv_display_delete(self.display);
            lv_style_reset(&mut *self.style_warning);
            lv_style_reset(&mut *self.style_critical);
            lv_deinit();
            INITIALIZED.store(false, Ordering::Release);
        }
        LIVE.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    static UI_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    #[test]
    fn moving_an_active_ui_preserves_styles_retained_by_lvgl() {
        let _guard = UI_TEST_LOCK.lock().unwrap();
        let settings = Settings::default();
        let packet = FastLink {
            esc_status: 1,
            esc_temp_mos_dC: 1500,
            ..FastLink::default()
        };
        let mut ui = Ui::with_settings(&settings);
        ui.update(&packet, &settings, false, 1000);
        unsafe {
            assert!(lv_color_eq(
                lv_obj_get_style_bg_color(ui.w.esc_temp_bg, 0),
                LVGL_RED()
            ));
        }
        #[inline(never)]
        fn relocate(ui: Ui) -> Box<Ui> {
            std::hint::black_box(Box::new(ui))
        }
        let mut moved = relocate(ui);
        moved.render_at(1033);
        unsafe {
            assert!(lv_color_eq(
                lv_obj_get_style_bg_color(moved.w.esc_temp_bg, 0),
                LVGL_RED()
            ));
        }
        moved.update(&packet, &settings, false, 1066);
        unsafe {
            assert!(lv_color_eq(
                lv_obj_get_style_bg_color(moved.w.esc_temp_bg, 0),
                LVGL_RED()
            ));
        }
    }
    #[test]
    fn production_abbreviations_preserve_sensor_id_and_direction() {
        assert_eq!(abbreviation(0, Level::WarnHigh), "MC-FT-H");
        assert_eq!(abbreviation(3, Level::CritHigh), "MC-MT-H");
        assert_eq!(abbreviation(28, Level::CritLow), "BC-T1-L");
        assert_eq!(abbreviation(25, Level::WarnHigh), "MC-LINK");
        assert_eq!(abbreviation(40, Level::CritHigh), "BC-LINK");
    }
    #[test]
    fn timed_feedback_missing_probes_and_rebuild_use_real_widgets() {
        let _guard = UI_TEST_LOCK.lock().unwrap();
        let mut settings = Settings::default();
        let mut packet = FastLink {
            bms_status: 1,
            esc_status: 1,
            device_state: 1,
            bms_soc: 72,
            bms_temp_sensors_C: [0; 8],
            ..FastLink::default()
        };
        let mut ui = Ui::with_settings(&settings);
        ui.set_battery_temperature(None);
        ui.update(&packet, &settings, false, 1000);
        unsafe {
            assert_eq!(
                CStr::from_ptr(lv_label_get_text(ui.w.batt_temp_label)).to_bytes(),
                b"-"
            );
        }
        ui.flash_arm_failure(1000);
        ui.flash_cruise(1000);
        ui.update(&packet, &settings, true, 1000);
        unsafe {
            assert!(!lv_obj_has_flag(
                ui.w.arm_fail_warning_icon_img,
                LV_OBJ_FLAG_HIDDEN
            ));
            assert!(!lv_obj_has_flag(ui.w.cruise_icon_img, LV_OBJ_FLAG_HIDDEN));
            assert!(!lv_obj_has_flag(ui.w.ble_pairing_icon, LV_OBJ_FLAG_HIDDEN));
        }
        ui.update(&packet, &settings, true, 1250);
        unsafe {
            assert!(lv_obj_has_flag(
                ui.w.arm_fail_warning_icon_img,
                LV_OBJ_FLAG_HIDDEN
            ));
        }
        ui.update(&packet, &settings, true, 1500);
        unsafe {
            assert!(lv_obj_has_flag(ui.w.ble_pairing_icon, LV_OBJ_FLAG_HIDDEN));
        }
        ui.update(&packet, &settings, false, 2500);
        unsafe {
            assert!(lv_obj_has_flag(
                ui.w.arm_fail_warning_icon_img,
                LV_OBJ_FLAG_HIDDEN
            ));
            assert!(lv_obj_has_flag(ui.w.cruise_icon_img, LV_OBJ_FLAG_HIDDEN));
        }
        packet.esc_critical_alerts = 1;
        ui.update(&packet, &settings, false, 3000);
        assert!(ui.vibration_due);
        ui.update(&packet, &settings, false, 3300);
        assert!(!ui.last_border_on);
        ui.update(&packet, &settings, false, 4000);
        assert!(ui.vibration_due);
        packet.esc_critical_alerts = 0;
        ui.update(&packet, &settings, false, 4100);
        assert!(!ui.last_border_on);
        // Recreate theme repeatedly; child handles and point buffers must be replaced,
        // with no dangling alert-widget pointers or growing LVGL style allocations.
        for i in 0..12 {
            settings.theme = (i % 2) as u8;
            ui.update(&packet, &settings, false, 4200 + i * 33);
            assert_eq!(unsafe { lv_obj_get_child_count(ui.w.critical_border) }, 4);
        }
    }
}
