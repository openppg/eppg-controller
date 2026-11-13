//! One owner of OTA flash handle; callbacks enqueue data, worker sends acknowledgements.
use controller_protocols::ota::{self, Command, Receive, Sectors};
use esp_idf_svc::sys;
pub struct Ota {
    handle: sys::esp_ota_handle_t,
    partition: *const sys::esp_partition_t,
    pub active: bool,
    sectors: Sectors,
}
impl Default for Ota {
    fn default() -> Self {
        Self {
            handle: 0,
            partition: std::ptr::null(),
            active: false,
            sectors: Sectors::default(),
        }
    }
}
impl Ota {
    pub fn abort(&mut self) {
        if self.active && self.handle != 0 {
            unsafe {
                sys::esp_ota_abort(self.handle);
            }
        }
        self.active = false;
        self.handle = 0;
        self.partition = std::ptr::null();
    }
    pub fn command(&mut self, data: &[u8], armed: bool, now: u32) -> Option<(Vec<u8>, bool)> {
        if let Some(ack) = ota::invalid_command_ack(data) {
            return Some((ack.to_vec(), false));
        }
        match ota::parse_command(data)? {
            Command::Start(total) => {
                self.abort();
                if armed {
                    return Some((ota::command_ack(1, 1).to_vec(), false));
                }
                self.partition =
                    unsafe { sys::esp_ota_get_next_update_partition(std::ptr::null()) };
                if self.partition.is_null() || total > unsafe { (*self.partition).size } {
                    return Some((ota::command_ack(1, 1).to_vec(), false));
                }
                let e =
                    unsafe { sys::esp_ota_begin(self.partition, total as usize, &mut self.handle) };
                if e != 0 {
                    return Some((ota::command_ack(1, 1).to_vec(), false));
                }
                self.active = true;
                self.sectors.start(total, now);
                Some((ota::command_ack(1, 0).to_vec(), false))
            }
            Command::End => {
                if !self.active || self.sectors.received != self.sectors.total {
                    self.abort();
                    return Some((ota::command_ack(2, 1).to_vec(), false));
                }
                let e = unsafe { sys::esp_ota_end(self.handle) };
                self.handle = 0;
                if e != 0 || unsafe { sys::esp_ota_set_boot_partition(self.partition) } != 0 {
                    self.abort();
                    return Some((ota::command_ack(2, 1).to_vec(), false));
                }
                self.active = false;
                Some((ota::command_ack(2, 0).to_vec(), true))
            }
        }
    }
    pub fn data(&mut self, data: &[u8], now: u32) -> Option<Vec<u8>> {
        if !self.active {
            return None;
        }
        match self.sectors.receive(data, now) {
            Receive::Complete(n) => {
                let sector = self.sectors.sector;
                let e = unsafe {
                    sys::esp_ota_write(self.handle, self.sectors.data(n).as_ptr().cast(), n)
                };
                if e == 0 {
                    self.sectors.written(n);
                    Some(ota::sector_ack(sector, 0, 0).to_vec())
                } else {
                    let ack = ota::sector_ack(sector, 2, self.sectors.sector).to_vec();
                    self.abort();
                    Some(ack)
                }
            }
            Receive::Rejected {
                sector,
                status,
                expected,
            } => Some(ota::sector_ack(sector, status, expected).to_vec()),
            _ => None,
        }
    }
    pub fn timeout(&mut self, now: u32) -> bool {
        if self.active && self.sectors.timed_out(now) {
            self.abort();
            true
        } else {
            false
        }
    }
}
impl Drop for Ota {
    fn drop(&mut self) {
        self.abort();
    }
}
