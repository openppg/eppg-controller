//! Reserved-flash whole-image staging. Erases and writes run on BLE worker;
//! the CAN owner reads one 256-byte window after upload is sealed.
use controller_esc::firmware::ImageStorage;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
unsafe extern "C" {
    fn eppg_stage_begin(size: u32) -> i32;
    fn eppg_stage_write(offset: u32, data: *const u8, len: usize) -> i32;
    fn eppg_stage_read(offset: u32, out: *mut u8, len: usize) -> i32;
    fn eppg_stage_finish() -> i32;
    fn eppg_stage_abort() -> i32;
}
static PREPARED: AtomicU32 = AtomicU32::new(0);
static SEALED: AtomicBool = AtomicBool::new(false);
static ABORT_PENDING: AtomicBool = AtomicBool::new(false);
pub fn begin(size: u32) -> bool {
    pump_abort();
    if ABORT_PENDING.load(Ordering::Acquire) {
        return false;
    }
    if unsafe { eppg_stage_begin(size) } != 0 {
        return false;
    }
    PREPARED.store(size, Ordering::Release);
    SEALED.store(false, Ordering::Release);
    true
}
pub fn write(offset: u32, data: &[u8]) -> Option<usize> {
    let n = unsafe { eppg_stage_write(offset, data.as_ptr(), data.len()) };
    if n >= 0 {
        Some(n as usize)
    } else {
        None
    }
}
pub fn finish() -> bool {
    if unsafe { eppg_stage_finish() } == 0 {
        SEALED.store(true, Ordering::Release);
        true
    } else {
        false
    }
}
pub fn abort() {
    let rc = unsafe { eppg_stage_abort() };
    PREPARED.store(0, Ordering::Release);
    SEALED.store(false, Ordering::Release);
    ABORT_PENDING.store(rc == -2, Ordering::Release);
}
pub fn pump_abort() {
    if ABORT_PENDING.load(Ordering::Acquire) {
        abort();
    }
}
#[derive(Debug, Default)]
pub struct StagedImage {
    size: usize,
    read_busy: bool,
}
impl ImageStorage for StagedImage {
    fn reserve(&mut self, size: usize) -> bool {
        if PREPARED.load(Ordering::Acquire) == size as u32 {
            self.size = size;
            true
        } else {
            false
        }
    }
    fn len(&self) -> usize {
        self.size
    }
    fn write(&mut self, offset: usize, data: &[u8]) -> bool {
        write(offset as u32, data).is_some()
    }
    fn read(&mut self, offset: usize, out: &mut [u8]) -> bool {
        self.read_busy = false;
        if offset > self.size || out.len() > self.size - offset {
            return false;
        }
        let rc = unsafe { eppg_stage_read(offset as u32, out.as_mut_ptr(), out.len()) };
        self.read_busy = rc == -2;
        rc == 0
    }
    fn read_retryable(&self) -> bool {
        self.read_busy
    }
    fn seal(&mut self) -> bool {
        SEALED.load(Ordering::Acquire)
    }
    fn clear(&mut self) {
        if self.size > 0 {
            abort();
            self.size = 0;
        }
    }
}
