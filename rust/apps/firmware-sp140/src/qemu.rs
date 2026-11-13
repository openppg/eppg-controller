//! Model inspection accessors; actual controller/tasks still execute on the S3 CPU.
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
static STATE: AtomicU32 = AtomicU32::new(0);
static LAST: AtomicU32 = AtomicU32::new(0);
static HANDLES: [AtomicUsize; 3] = [
    AtomicUsize::new(0),
    AtomicUsize::new(0),
    AtomicUsize::new(0),
];
pub fn set_state(state: u32, now: u32) {
    STATE.store(state, Ordering::Relaxed);
    LAST.store(now, Ordering::Relaxed);
}
pub fn set_handle(kind: usize) {
    HANDLES[kind].store(
        unsafe { esp_idf_svc::sys::xTaskGetCurrentTaskHandle() } as usize,
        Ordering::Release,
    );
}
#[no_mangle]
pub extern "C" fn eppg_qemu_state() -> u32 {
    STATE.load(Ordering::Relaxed)
}
#[no_mangle]
pub extern "C" fn eppg_qemu_last_throttle() -> u32 {
    LAST.load(Ordering::Relaxed)
}
#[no_mangle]
pub extern "C" fn eppg_qemu_task_handle(kind: i32) -> *mut std::ffi::c_void {
    HANDLES
        .get(kind as usize)
        .map(|h| h.load(Ordering::Acquire))
        .unwrap_or(0) as *mut std::ffi::c_void
}

unsafe extern "C" {
    fn eppg_qemu_boot_complete();
}
pub fn boot_complete() {
    unsafe {
        eppg_qemu_boot_complete();
    }
    crate::platform::console("Rust controller boot complete\n");
}
