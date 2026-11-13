//! Thin IDF C-ABI peripheral boundary plus native Rust chip drivers.
use esp_idf_svc::sys;
use std::ffi::c_void;
unsafe extern "C" {
    pub fn eppg_millis() -> u32;
    pub fn eppg_micros() -> u64;
    pub fn eppg_delay_ms(ms: u32);
    pub fn eppg_gpio_config(pin: i32, output: bool, pullup: bool) -> i32;
    pub fn eppg_gpio_read(pin: i32) -> i32;
    pub fn eppg_gpio_write(pin: i32, value: i32);
    pub fn eppg_adc_read(pin: i32) -> i32;
    pub fn eppg_pwm_config(channel: i32, pin: i32, hz: u32) -> i32;
    pub fn eppg_pwm_write(channel: i32, duty: u32);
    pub fn eppg_pwm_frequency(channel: i32, hz: u32);
    pub fn eppg_pixel_write(pin: i32, rgb: u32) -> i32;
    pub fn eppg_spi_bus_init(mosi: i32, miso: i32, sclk: i32) -> i32;
    pub fn eppg_spi_add_device(cs: i32, hz: u32, mode: u8) -> *mut c_void;
    pub fn eppg_spi_transfer(device: *mut c_void, tx: *const u8, rx: *mut u8, len: usize) -> i32;
    pub fn eppg_i2c_init(sda: i32, scl: i32) -> i32;
    pub fn eppg_usb_init() -> i32;
    pub fn eppg_usb_read(data: *mut u8, len: usize) -> i32;
    pub fn eppg_usb_write(data: *const u8, len: usize) -> i32;
    pub fn eppg_cpu_temperature() -> f32;
}
pub fn now() -> u32 {
    unsafe { eppg_millis() }
}
pub fn sleep(ms: u32) {
    unsafe {
        eppg_delay_ms(ms);
    }
}
pub fn console(text: &str) {
    for chunk in text.as_bytes().chunks(96) {
        let mut at = 0;
        while at < chunk.len() {
            let n = unsafe { eppg_usb_write(chunk[at..].as_ptr(), chunk.len() - at) };
            if n <= 0 {
                return;
            } // USB disconnected/full: best-effort log must never block a worker.
            at += n as usize;
        }
    }
}
pub fn init() {
    unsafe {
        eppg_usb_init();
        eppg_gpio_config(1, false, true);
        eppg_spi_bus_init(11, 13, 12);
        eppg_i2c_init(44, 41);
        eppg_gpio_config(42, true, false);
        eppg_gpio_write(42, 1);
        eppg_gpio_config(40, true, false);
        eppg_gpio_write(40, 1);
        eppg_pwm_config(0, 46, 1000);
        eppg_pwm_config(1, 8, 1000);
    }
}
pub struct Spi {
    handle: *mut c_void,
}
impl Spi {
    pub fn new(cs: i32, hz: u32) -> Option<Self> {
        let handle = unsafe { eppg_spi_add_device(cs, hz, 0) };
        if handle.is_null() {
            None
        } else {
            Some(Self { handle })
        }
    }
    pub fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
        if rx.len() != tx.len() {
            return false;
        }
        unsafe { eppg_spi_transfer(self.handle, tx.as_ptr(), rx.as_mut_ptr(), tx.len()) == 0 }
    }
    pub fn write(&mut self, data: &[u8]) -> bool {
        unsafe {
            eppg_spi_transfer(self.handle, data.as_ptr(), std::ptr::null_mut(), data.len()) == 0
        }
    }
}
pub struct Mcp2515 {
    spi: Spi,
}
impl Mcp2515 {
    pub fn new() -> Option<Self> {
        let mut m = Self {
            spi: Spi::new(5, 10_000_000)?,
        };
        m.spi.write(&[0xc0]);
        sleep(10);
        m.write(0x0f, &[0x80]);
        if m.read(0x0e) & 0xe0 != 0x80 {
            return None;
        }
        // Adafruit16MHz 250kbps CNF1/2/3, 75% sample point.
        m.write(0x2a, &[0x41]);
        m.write(0x29, &[0xf1]);
        m.write(0x28, &[0x85]);
        m.write(0x60, &[0x60]);
        m.write(0x70, &[0x60]);
        m.write(0x2b, &[3]);
        m.write(0x2c, &[0]);
        m.write(0x0f, &[0]);
        Some(m)
    }
    fn read(&mut self, reg: u8) -> u8 {
        let mut rx = [0; 3];
        self.spi.transfer(&[3, reg, 0], &mut rx);
        rx[2]
    }
    fn write(&mut self, reg: u8, data: &[u8]) {
        let mut tx = Vec::with_capacity(2 + data.len());
        tx.extend_from_slice(&[2, reg]);
        tx.extend_from_slice(data);
        self.spi.write(&tx);
    }
    pub fn receive(&mut self) -> Option<(u32, [u8; 8], usize)> {
        let status = self.read(0x2c);
        let (cmd, mask) = if status & 1 != 0 {
            (0x90, 1)
        } else if status & 2 != 0 {
            (0x94, 2)
        } else {
            return None;
        };
        let mut rx = [0; 14];
        let mut tx = [0; 14];
        tx[0] = cmd;
        self.spi.transfer(&tx, &mut rx);
        self.spi.write(&[5, 0x2c, mask, 0]);
        let h = &rx[1..];
        let id = if h[1] & 8 != 0 {
            ((h[0] as u32) << 21)
                | ((h[1] as u32 & 0xe0) << 13)
                | ((h[1] as u32 & 3) << 16)
                | ((h[2] as u32) << 8)
                | h[3] as u32
        } else {
            ((h[0] as u32) << 3) | (h[1] >> 5) as u32
        };
        let n = (h[4] & 15).min(8) as usize;
        let mut data = [0; 8];
        data[..n].copy_from_slice(&h[5..5 + n]);
        Some((id, data, n))
    }
}
/// ST7735 BLACKTAB 160x128 panel; all screen composition is in controller-ui.
pub struct Panel {
    spi: Spi,
    rotation: u8,
}
impl Panel {
    pub fn new(rotation: u8) -> Option<Self> {
        unsafe {
            eppg_gpio_config(14, true, false);
            eppg_gpio_config(15, true, false);
            eppg_gpio_write(15, 0);
        }
        sleep(100);
        unsafe {
            eppg_gpio_write(15, 1);
        }
        sleep(120);
        let mut p = Self {
            spi: Spi::new(10, 40_000_000)?,
            rotation,
        };
        p.command(1, &[]);
        sleep(150);
        p.command(0x11, &[]);
        sleep(500);
        p.command(0xb1, &[1, 0x2c, 0x2d]);
        p.command(0xb2, &[1, 0x2c, 0x2d]);
        p.command(0xb3, &[1, 0x2c, 0x2d, 1, 0x2c, 0x2d]);
        p.command(0xb4, &[7]);
        p.command(0xc0, &[0xa2, 2, 0x84]);
        p.command(0xc1, &[0xc5]);
        p.command(0xc2, &[0x0a, 0]);
        p.command(0xc3, &[0x8a, 0x2a]);
        p.command(0xc4, &[0x8a, 0xee]);
        p.command(0xc5, &[0x0e]);
        p.command(0x20, &[]);
        p.command(0x3a, &[5]);
        p.set_rotation(rotation);
        p.command(
            0xe0,
            &[
                2, 0x1c, 7, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2b, 0x39, 0, 1, 3, 0x10,
            ],
        );
        p.command(
            0xe1,
            &[
                3, 0x1d, 7, 6, 0x2e, 0x2c, 0x29, 0x2d, 0x2e, 0x2e, 0x37, 0x3f, 0, 0, 2, 0x10,
            ],
        );
        p.command(0x13, &[]);
        sleep(10);
        p.command(0x29, &[]);
        sleep(100);
        Some(p)
    }
    fn command(&mut self, cmd: u8, data: &[u8]) {
        unsafe {
            eppg_gpio_write(14, 0);
        }
        self.spi.write(&[cmd]);
        if !data.is_empty() {
            unsafe {
                eppg_gpio_write(14, 1);
            }
            self.spi.write(data);
        }
    }
    pub fn set_rotation(&mut self, rotation: u8) {
        self.rotation = rotation;
        self.command(0x36, &[if rotation == 1 { 0xa0 } else { 0x60 }]);
    }
    pub fn flush(&mut self, x1: i32, y1: i32, x2: i32, y2: i32, pixels: &[u8]) {
        self.command(0x2a, &[0, x1 as u8, 0, x2 as u8]);
        self.command(0x2b, &[0, y1 as u8, 0, y2 as u8]);
        self.command(0x2c, &[]);
        unsafe {
            eppg_gpio_write(14, 1);
        }
        let mut chunk = [0u8; 1024];
        for data in pixels.chunks(1024) {
            for (i, p) in data.chunks_exact(2).enumerate() {
                chunk[i * 2] = p[1];
                chunk[i * 2 + 1] = p[0];
            }
            self.spi.write(&chunk[..data.len()]);
        }
    }
}
pub fn twai_init() -> bool {
    unsafe {
        let config = sys::twai_general_config_t {
            tx_io: 2,
            rx_io: 3,
            mode: sys::twai_mode_t_TWAI_MODE_NORMAL,
            tx_queue_len: 64,
            rx_queue_len: 128,
            clkout_io: -1,
            bus_off_io: -1,
            alerts_enabled: 0,
            ..Default::default()
        };
        let timing = sys::twai_timing_config_t {
            brp: 4,
            tseg_1: 15,
            tseg_2: 4,
            sjw: 3,
            triple_sampling: false,
            ..Default::default()
        };
        let filter = sys::twai_filter_config_t {
            acceptance_code: 0,
            acceptance_mask: u32::MAX,
            single_filter: true,
        };
        sys::twai_driver_install(&config, &timing, &filter) == 0 && sys::twai_start() == 0
    }
}
pub fn can_receive() -> Option<controller_dronecan::Frame> {
    for _ in 0..8 {
        let mut m: sys::twai_message_t = unsafe { std::mem::zeroed() };
        if unsafe { sys::twai_receive(&mut m, 0) } != 0 {
            return None;
        }
        let flags = unsafe { m.__bindgen_anon_1.__bindgen_anon_1 };
        if !controller_protocols::wireless::dronecan_frame_allowed(
            flags.extd() != 0,
            flags.rtr() != 0,
            m.data_length_code,
        ) {
            continue;
        }
        return Some(controller_dronecan::Frame {
            id: m.identifier,
            data: m.data,
            len: m.data_length_code,
        });
    }
    None
}

pub fn can_send(f: controller_dronecan::Frame) -> bool {
    let mut m: sys::twai_message_t = unsafe { std::mem::zeroed() };
    m.identifier = f.id;
    m.data = f.data;
    m.data_length_code = f.len;
    unsafe {
        m.__bindgen_anon_1.__bindgen_anon_1.set_extd(1);
        sys::twai_transmit(&m, 0) == 0
    }
}
pub fn can_health() {
    unsafe {
        let mut status: sys::twai_status_info_t = std::mem::zeroed();
        if sys::twai_get_status_info(&mut status) == 0 {
            if status.state == sys::twai_state_t_TWAI_STATE_BUS_OFF {
                sys::twai_initiate_recovery();
            } else if status.state == sys::twai_state_t_TWAI_STATE_STOPPED {
                sys::twai_start();
            }
        }
    }
}
