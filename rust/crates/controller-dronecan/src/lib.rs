//! Single-owner DroneCAN v0 transport using pinned libcanard C implementation.
use std::{ffi::c_void, ptr::NonNull};
unsafe extern "C" {
    fn dc_new() -> *mut c_void;
    fn dc_free(ptr: *mut c_void);
    fn dc_node(ptr: *mut c_void, node: u8);
    fn dc_register(ptr: *mut c_void, id: u16, kind: u8, source: u8, signature: u64) -> i32;
    fn dc_rx(ptr: *mut c_void, id: u32, data: *const u8, len: u8, now: u64) -> i32;
    fn dc_tx(ptr: *mut c_void, id: *mut u32, data: *mut u8, len: *mut u8) -> i32;
    fn dc_tx_pop(ptr: *mut c_void);
    fn dc_next_size(ptr: *mut c_void) -> i32;
    fn dc_take(
        ptr: *mut c_void,
        id: *mut u16,
        kind: *mut u8,
        source: *mut u8,
        tid: *mut u8,
        data: *mut u8,
        cap: u16,
    ) -> i32;
    fn dc_request(
        ptr: *mut c_void,
        destination: u8,
        id: u16,
        signature: u64,
        tid: *mut u8,
        data: *const u8,
        len: u16,
    ) -> i32;
    fn dc_respond(
        ptr: *mut c_void,
        destination: u8,
        id: u16,
        signature: u64,
        tid: u8,
        data: *const u8,
        len: u16,
    ) -> i32;
    fn dc_cleanup(ptr: *mut c_void, now: u64);
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Frame {
    pub id: u32,
    pub data: [u8; 8],
    pub len: u8,
}
#[derive(Clone, Debug)]
pub struct Transfer {
    pub id: u16,
    pub kind: u8,
    pub source: u8,
    pub transfer_id: u8,
    pub data: Vec<u8>,
}
/// !Sync/!Send: created, used and dropped on its sole CAN-owner task.
pub struct DroneCan {
    ptr: NonNull<c_void>,
    _owner: std::marker::PhantomData<std::rc::Rc<()>>,
}
impl DroneCan {
    pub fn new(node: u8) -> Option<Self> {
        let ptr = NonNull::new(unsafe { dc_new() })?;
        let mut out = Self {
            ptr,
            _owner: std::marker::PhantomData,
        };
        out.set_node(node);
        Some(out)
    }
    pub fn set_node(&mut self, node: u8) {
        unsafe {
            dc_node(self.ptr.as_ptr(), node);
        }
    }
    pub fn register(&mut self, id: u16, kind: u8, source: u8, signature: u64) -> bool {
        unsafe { dc_register(self.ptr.as_ptr(), id, kind, source, signature) == 0 }
    }
    pub fn receive(&mut self, frame: Frame, now_us: u64) -> i32 {
        if frame.len > 8 {
            return -1;
        }
        unsafe {
            dc_rx(
                self.ptr.as_ptr(),
                frame.id,
                frame.data.as_ptr(),
                frame.len,
                now_us,
            )
        }
    }
    pub fn peek_frame(&mut self) -> Option<Frame> {
        let mut f = Frame::default();
        if unsafe {
            dc_tx(
                self.ptr.as_ptr(),
                &mut f.id,
                f.data.as_mut_ptr(),
                &mut f.len,
            )
        } == 1
        {
            Some(f)
        } else {
            None
        }
    }
    pub fn pop_frame(&mut self) {
        unsafe {
            dc_tx_pop(self.ptr.as_ptr());
        }
    }
    pub fn take_transfer(&mut self) -> Option<Transfer> {
        let size = unsafe { dc_next_size(self.ptr.as_ptr()) };
        if size < 0 {
            return None;
        }
        let mut data = Vec::new();
        data.try_reserve_exact(size as usize).ok()?;
        data.resize(size as usize, 0);
        let mut t = Transfer {
            id: 0,
            kind: 0,
            source: 0,
            transfer_id: 0,
            data,
        };
        let n = unsafe {
            dc_take(
                self.ptr.as_ptr(),
                &mut t.id,
                &mut t.kind,
                &mut t.source,
                &mut t.transfer_id,
                t.data.as_mut_ptr(),
                size as u16,
            )
        };
        if n < 0 {
            return None;
        }
        t.data.truncate(n as usize);
        Some(t)
    }
    pub fn request(
        &mut self,
        destination: u8,
        id: u16,
        signature: u64,
        tid: &mut u8,
        data: &[u8],
    ) -> i32 {
        if data.len() > 1023 {
            return -1;
        }
        unsafe {
            dc_request(
                self.ptr.as_ptr(),
                destination,
                id,
                signature,
                tid,
                data.as_ptr(),
                data.len() as u16,
            )
        }
    }
    pub fn respond(
        &mut self,
        destination: u8,
        id: u16,
        signature: u64,
        tid: u8,
        data: &[u8],
    ) -> i32 {
        if data.len() > 1023 {
            return -1;
        }
        unsafe {
            dc_respond(
                self.ptr.as_ptr(),
                destination,
                id,
                signature,
                tid,
                data.as_ptr(),
                data.len() as u16,
            )
        }
    }
    pub fn cleanup(&mut self, now_us: u64) {
        unsafe {
            dc_cleanup(self.ptr.as_ptr(), now_us);
        }
    }
}
impl Drop for DroneCan {
    fn drop(&mut self) {
        unsafe {
            dc_free(self.ptr.as_ptr());
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn deliver(sender: &mut DroneCan, receiver: &mut DroneCan, now: u64) {
        while let Some(frame) = sender.peek_frame() {
            assert!(receiver.receive(frame, now) >= 0);
            sender.pop_frame();
        }
    }
    #[test]
    fn completed_payloads_survive_wrap_overflow_and_empty_poll() {
        let mut sender = DroneCan::new(1).unwrap();
        let mut receiver = DroneCan::new(32).unwrap();
        assert!(receiver.register(217, 1, 1, 0x30c3f78df78cbee5));
        let mut tid = 0;
        // Queue capacity and maximum payload support remain unchanged.
        for batch in 0..3 {
            for n in 0..17u8 {
                let payload = vec![n; 1023];
                assert!(sender.request(32, 217, 0x30c3f78df78cbee5, &mut tid, &payload) > 0);
                deliver(
                    &mut sender,
                    &mut receiver,
                    1 + (batch * 17 + u64::from(n)) * 10_000,
                );
            }
            for n in 0..16u8 {
                let transfer = receiver.take_transfer().unwrap();
                assert_eq!(transfer.id, 217);
                assert_eq!(transfer.source, 1);
                assert_eq!(transfer.data, vec![n; 1023]);
            }
            assert!(receiver.take_transfer().is_none());
        }
        assert!(sender.request(32, 217, 0x30c3f78df78cbee5, &mut tid, &[]) > 0);
        deliver(&mut sender, &mut receiver, 600_000);
        assert!(receiver.take_transfer().unwrap().data.is_empty());
        // Drop also owns any queued payloads that were never taken.
        assert!(sender.request(32, 217, 0x30c3f78df78cbee5, &mut tid, &[42; 128]) > 0);
        deliver(&mut sender, &mut receiver, 610_000);
    }
    #[test]
    fn request_frame_id_and_tail() {
        let mut c = DroneCan::new(1).unwrap();
        let mut tid = 0;
        assert!(c.request(0x20, 217, 0x30c3f78df78cbee5, &mut tid, &[0x70, 0x17, 0]) > 0);
        let f = c.peek_frame().unwrap();
        assert_eq!(f.id & 0x7f, 1);
        assert_eq!((f.id >> 8) & 0x7f, 0x20);
        assert_eq!((f.id >> 16) & 255, 217);
        assert_eq!(f.len, 4);
        assert_eq!(f.data[3] & 0xc0, 0xc0);
        c.pop_frame();
        assert!(c.peek_frame().is_none());
        assert_eq!(tid, 1);
    }
}
