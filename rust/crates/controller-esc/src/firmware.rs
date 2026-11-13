use super::{le16, restart, Request};
use controller_core::State;
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Phase {
    #[default]
    Idle = 0,
    Receiving = 1,
    Restarting = 2,
    Checking = 3,
    Starting = 4,
    Sending = 5,
    Ending = 6,
    Done = 7,
    Failed = 8,
}
/// Whole-image staging, decoupled from the 256-byte CAN transfer window.
/// Hardware uses the reserved flash scratch area; host tests use memory.
pub trait ImageStorage: std::fmt::Debug {
    fn reserve(&mut self, size: usize) -> bool;
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn write(&mut self, offset: usize, data: &[u8]) -> bool;
    fn read(&mut self, offset: usize, out: &mut [u8]) -> bool;
    /// Last read was temporarily busy; retry without failing the session.
    fn read_retryable(&self) -> bool {
        false
    }
    fn seal(&mut self) -> bool {
        true
    }
    fn clear(&mut self);
}
#[derive(Debug, Default)]
pub struct MemoryImage(Vec<u8>);
impl ImageStorage for MemoryImage {
    fn reserve(&mut self, size: usize) -> bool {
        self.0.clear();
        if self.0.try_reserve_exact(size).is_err() {
            return false;
        }
        self.0.resize(size, 0);
        true
    }
    fn len(&self) -> usize {
        self.0.len()
    }
    fn write(&mut self, offset: usize, data: &[u8]) -> bool {
        if offset > self.0.len() || data.len() > self.0.len() - offset {
            return false;
        }
        self.0[offset..offset + data.len()].copy_from_slice(data);
        true
    }
    fn read(&mut self, offset: usize, out: &mut [u8]) -> bool {
        if offset > self.0.len() || out.len() > self.0.len() - offset {
            return false;
        }
        out.copy_from_slice(&self.0[offset..offset + out.len()]);
        true
    }
    fn clear(&mut self) {
        self.0 = Vec::new();
    }
}
#[derive(Debug)]
pub struct Flasher {
    pub code: u8,
    pub phase: Phase,
    pub progress: u16,
    image: Box<dyn ImageStorage>,
    received: usize,
    last_rx: u32,
    last_send: u32,
    read_pending: bool,
    started: u32,
    retries: u8,
    index: u16,
    chunks: u16,
    response: Option<(u16, Vec<u8>)>,
}
impl Default for Flasher {
    fn default() -> Self {
        Self::with_storage(Box::<MemoryImage>::default())
    }
}
impl Flasher {
    pub fn with_storage(image: Box<dyn ImageStorage>) -> Self {
        Self {
            code: 0,
            phase: Phase::Idle,
            progress: 0,
            image,
            received: 0,
            last_rx: 0,
            last_send: 0,
            read_pending: false,
            started: 0,
            retries: 0,
            index: 0,
            chunks: 0,
            response: None,
        }
    }

    pub fn active(&self) -> bool {
        !matches!(self.phase, Phase::Idle | Phase::Done | Phase::Failed)
    }
    pub fn status(&self) -> [u8; 4] {
        [
            self.code,
            self.phase as u8,
            self.progress as u8,
            (self.progress >> 8) as u8,
        ]
    }
    fn fail(&mut self, code: u8) {
        self.image.clear();
        self.code = code;
        self.phase = Phase::Failed;
    }
    pub fn reject(&mut self, code: u8) {
        if self.active() {
            self.code = code;
        } else {
            self.fail(code);
        }
    }
    pub fn abort(&mut self) {
        self.fail(0x80);
    }
    pub fn begin(
        &mut self,
        hw: u16,
        size: u32,
        connected_hw: u16,
        state: State,
        busy: bool,
        now: u32,
    ) -> bool {
        if state != State::Disarmed {
            self.fail(0x81);
            return false;
        }
        if self.active() || busy || !(32..=262144).contains(&size) {
            self.code = 0x84;
            return false;
        }
        if connected_hw == 0 || hw != connected_hw {
            self.fail(0x83);
            return false;
        }
        if !self.image.reserve(size as usize) {
            self.fail(0x84);
            return false;
        }
        self.received = 0;
        self.code = 1;
        self.phase = Phase::Receiving;
        self.progress = 0;
        self.last_rx = now;
        true
    }
    pub fn chunk(&mut self, offset: u32, data: &[u8], now: u32) -> Option<usize> {
        let offset = offset as usize;
        if self.phase != Phase::Receiving
            || offset > self.image.len()
            || data.len() > self.image.len() - offset
        {
            return None;
        }
        if !self.image.write(offset, data) {
            self.fail(0x80);
            return None;
        }
        self.record_chunk(offset as u32, data.len(), now)
    }
    /// Track bytes already accepted by the external staging worker. The worker
    /// must successfully persist the exact range before sending this event.
    pub fn record_chunk(&mut self, offset: u32, len: usize, now: u32) -> Option<usize> {
        let offset = offset as usize;
        if self.phase != Phase::Receiving
            || offset > self.image.len()
            || len > self.image.len() - offset
        {
            return None;
        }

        if offset <= self.received && offset + len > self.received {
            self.received = offset + len;
        }
        self.last_rx = now;
        self.progress = (self.received * 500 / self.image.len()) as u16;
        Some(self.received)
    }
    pub fn end(&mut self, now: u32) -> bool {
        if self.phase != Phase::Receiving
            || self.received != self.image.len()
            || self.image.len() < 32
        {
            return false;
        }
        if !self.image.seal() {
            self.fail(0x80);
            return false;
        }
        self.chunks = ((self.image.len() - 32).div_ceil(256)) as u16;
        self.index = 0;
        self.retries = 0;
        self.response = None;
        self.phase = Phase::Restarting;
        self.started = now;
        self.last_send = 0;
        true
    }
    pub fn response(&mut self, id: u16, d: &[u8]) {
        let expected = match self.phase {
            Phase::Checking => 200,
            Phase::Starting => 201,
            Phase::Sending => 202,
            Phase::Ending => 203,
            _ => 0,
        };
        if id == expected {
            self.response = Some((id, d.to_vec()));
        }
    }
    fn request(&mut self, now: u32) -> Option<Request> {
        self.read_pending = false;
        self.response = None;
        self.last_send = now;
        Some(match self.phase {
            Phase::Restarting => restart(),
            Phase::Checking => Request {
                id: 200,
                payload: vec![],
            },
            Phase::Starting => {
                let mut header = [0u8; 8];
                if !self.image.read(0, &mut header) {
                    if self.image.read_retryable() {
                        self.read_pending = true;
                    } else {
                        self.fail(0x80);
                    }
                    return None;
                }
                let mut payload = header[..2].to_vec();
                payload.extend_from_slice(&header[4..8]);
                Request { id: 201, payload }
            }
            Phase::Sending => {
                let mut payload = self.index.to_le_bytes().to_vec();
                payload.resize(258, 255);
                let off = 32 + self.index as usize * 256;
                if off < self.image.len() {
                    let n = (self.image.len() - off).min(256);
                    if !self.image.read(off, &mut payload[2..2 + n]) {
                        if self.image.read_retryable() {
                            self.read_pending = true;
                        } else {
                            self.fail(0x80);
                        }
                        return None;
                    }
                }
                Request { id: 202, payload }
            }
            _ => Request {
                id: 203,
                payload: vec![],
            },
        })
    }
    pub fn tick(&mut self, now: u32, state: State) -> Vec<Request> {
        if !self.active() {
            return vec![];
        }
        if self.phase == Phase::Receiving {
            if now.wrapping_sub(self.last_rx) > 15000 {
                self.fail(0x82);
            }
            return vec![];
        }
        if state != State::Disarmed {
            self.fail(0x81);
            return vec![];
        }
        if self.read_pending {
            return self.request(now).into_iter().collect();
        }
        let response = self.response.take();
        match self.phase {
            Phase::Restarting => {
                if self.last_send == 0 {
                    self.started = now;
                    self.code = 2;
                    return self.request(now).into_iter().collect();
                }
                if now.wrapping_sub(self.started) >= 700 {
                    self.phase = Phase::Checking;
                    self.retries = 0;
                    return self.request(now).into_iter().collect();
                }
                return vec![];
            }
            Phase::Checking => {
                if let Some((_, d)) = response {
                    if d.first() == Some(&2) {
                        self.phase = Phase::Starting;
                        self.retries = 0;
                        return self.request(now).into_iter().collect();
                    }
                    if self.retries < 10 {
                        self.retries += 1;
                        self.phase = Phase::Restarting;
                        self.started = now;
                        return self.request(now).into_iter().collect();
                    }
                    self.fail(0x85);
                    return vec![];
                }
            }
            Phase::Starting => {
                if let Some((_, d)) = response {
                    if d.first() == Some(&0) {
                        self.phase = Phase::Sending;
                        self.index = 0;
                        self.retries = 0;
                        self.code = 3;
                        self.progress = 500;
                        return self.request(now).into_iter().collect();
                    }
                    self.fail(0x85);
                    return vec![];
                }
            }
            Phase::Sending => {
                if let Some((_, d)) = response {
                    if d.len() >= 3 && d[0] == 0 {
                        self.index = le16(&d, 1);
                        self.retries = 0;
                        self.progress = if self.chunks == 0 {
                            1000
                        } else {
                            500 + (self.index as u32 * 500 / self.chunks as u32) as u16
                        };
                        if self.index >= self.chunks {
                            self.phase = Phase::Ending;
                        }
                        return self.request(now).into_iter().collect();
                    }
                    if self.retries < 10 {
                        self.retries += 1;
                        return self.request(now).into_iter().collect();
                    }
                    self.fail(0x85);
                    return vec![];
                }
            }
            Phase::Ending => {
                if let Some((_, d)) = response {
                    if d.first() == Some(&0) {
                        self.image.clear();
                        self.code = 4;
                        self.phase = Phase::Done;
                        self.progress = 1000;
                    } else {
                        self.fail(0x85);
                    }
                    return vec![];
                }
            }
            _ => {}
        }
        if self.last_send != 0 && now.wrapping_sub(self.last_send) > 150 {
            if self.retries < 10 {
                self.retries += 1;
                return self.request(now).into_iter().collect();
            }
            self.fail(0x82);
        }
        vec![]
    }
}
