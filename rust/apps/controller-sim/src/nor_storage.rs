//! Test backend for the exact C staging implementation used by S3 firmware.
use controller_esc::firmware::ImageStorage;
use std::ffi::c_void;
unsafe extern "C" {
    fn eppg_host_stage_new() -> *mut c_void;
    fn eppg_host_stage_delete(p: *mut c_void);
    fn eppg_host_stage_begin(p: *mut c_void, n: u32) -> bool;
    fn eppg_host_stage_write(p: *mut c_void, o: u32, b: *const u8, n: usize) -> bool;
    fn eppg_host_stage_read(p: *mut c_void, o: u32, b: *mut u8, n: usize) -> bool;
    fn eppg_host_stage_seal(p: *mut c_void) -> bool;
    fn eppg_host_stage_abort(p: *mut c_void);
    fn eppg_host_stage_ram() -> usize;
}
#[derive(Debug)]
pub struct NorStorage {
    handle: *mut c_void,
    len: usize,
}
impl NorStorage {
    pub fn new() -> Self {
        let handle = unsafe { eppg_host_stage_new() };
        assert!(!handle.is_null());
        assert!(unsafe { eppg_host_stage_ram() } < 4224);
        Self { handle, len: 0 }
    }
}
impl Drop for NorStorage {
    fn drop(&mut self) {
        unsafe { eppg_host_stage_delete(self.handle) }
    }
}
impl ImageStorage for NorStorage {
    fn reserve(&mut self, size: usize) -> bool {
        self.clear();
        let Ok(n) = u32::try_from(size) else {
            return false;
        };
        if unsafe { eppg_host_stage_begin(self.handle, n) } {
            self.len = size;
            true
        } else {
            false
        }
    }
    fn len(&self) -> usize {
        self.len
    }
    fn write(&mut self, offset: usize, bytes: &[u8]) -> bool {
        let Ok(offset) = u32::try_from(offset) else {
            return false;
        };
        unsafe { eppg_host_stage_write(self.handle, offset, bytes.as_ptr(), bytes.len()) }
    }
    fn read(&mut self, offset: usize, bytes: &mut [u8]) -> bool {
        let Ok(offset) = u32::try_from(offset) else {
            return false;
        };
        unsafe { eppg_host_stage_read(self.handle, offset, bytes.as_mut_ptr(), bytes.len()) }
    }
    fn seal(&mut self) -> bool {
        unsafe { eppg_host_stage_seal(self.handle) }
    }
    fn clear(&mut self) {
        unsafe { eppg_host_stage_abort(self.handle) };
        self.len = 0;
    }
}
