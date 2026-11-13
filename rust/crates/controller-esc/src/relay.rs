use super::{le16, restart, Request};
use controller_core::State;
include!(concat!(env!("OUT_DIR"), "/params.rs"));
const UNLOCK: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/unlock.bin"));
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Phase {
    #[default]
    Idle = 0,
    Unlock = 1,
    Write = 2,
    Save = 3,
    Restart = 4,
    WaitReboot = 5,
    Verify = 6,
    Reading = 7,
    BatchWrite = 8,
    BatchVerify = 9,
    Done = 10,
    Failed = 11,
}
#[derive(Clone, Debug, Default)]
pub struct Status {
    pub code: u8,
    pub phase: Phase,
    pub id: u16,
    pub detail: u8,
    pub readback: Vec<u8>,
}
impl Status {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![
            self.code,
            self.id as u8,
            (self.id >> 8) as u8,
            self.phase as u8,
            self.detail,
            0,
            0,
        ];
        out.extend_from_slice(&self.readback[..self.readback.len().min(8)]);
        out
    }
}
#[derive(Clone, Debug)]
pub struct Relay {
    pub status: Status,
    pub result: Vec<u8>,
    pub require_unlock: bool,
    pub staged: Vec<(u16, Vec<u8>)>,
    work: Vec<(u16, Vec<u8>)>,
    index: usize,
    reading: bool,
    batch: bool,
    retries: u8,
    last_send: u32,
    phase_start: u32,
    last_probe: u32,
    reunlocked: bool,
    mismatches: u16,
    terminal: Option<u32>,
    response: Option<(u16, Vec<u8>)>,
    start_pending: bool,
}
impl Default for Relay {
    fn default() -> Self {
        Self {
            status: Status::default(),
            result: Vec::with_capacity(3072),
            require_unlock: true,
            staged: Vec::new(),
            work: Vec::new(),
            index: 0,
            reading: false,
            batch: false,
            retries: 0,
            last_send: 0,
            phase_start: 0,
            last_probe: 0,
            reunlocked: false,
            mismatches: 0,
            terminal: None,
            response: None,
            start_pending: false,
        }
    }
}
impl Relay {
    pub fn active(&self) -> bool {
        self.start_pending
            || !matches!(self.status.phase, Phase::Idle | Phase::Done | Phase::Failed)
    }
    pub fn settle(&self, now: u32) -> bool {
        self.terminal
            .map(|at| now.wrapping_sub(at) < 9000)
            .unwrap_or(false)
    }
    pub fn response(&mut self, id: u16, data: &[u8]) {
        let expected = match self.status.phase {
            Phase::Unlock => 225,
            Phase::Write | Phase::BatchWrite => 220,
            Phase::Save => 222,
            Phase::WaitReboot if self.require_unlock && !self.reunlocked => 225,
            Phase::WaitReboot | Phase::Verify | Phase::Reading | Phase::BatchVerify => 221,
            _ => 0,
        };
        if id == expected {
            self.response = Some((id, data.to_vec()));
        }
    }
    pub fn command(&mut self, op: u8, data: &[u8], busy: bool) -> bool {
        if busy || self.active() {
            return false;
        }
        match op {
            0x40 => {
                self.staged.clear();
                return true;
            }
            0x41 => {
                if data.len() < 3 {
                    return false;
                }
                let n = data[2] as usize;
                if n == 0
                    || n > 48
                    || data.len() < 3 + n
                    || self.staged.iter().map(|(_, d)| d.len() + 3).sum::<usize>() + 3 + n > 2560
                {
                    return false;
                }
                self.staged.push((le16(data, 0), data[3..3 + n].to_vec()));
                return true;
            }
            0x42 => {
                if self.staged.is_empty() {
                    return false;
                }
                self.work = self.staged.clone();
                self.batch = true;
                self.reading = false;
            }
            0x30 => {
                self.work = PARAM_IDS.iter().map(|id| (*id, vec![])).collect();
                self.reading = true;
                self.batch = false;
                self.result.clear();
            }
            0x10 | 0x1f => {
                if data.len() < 3 {
                    return false;
                }
                let n = data[2] as usize;
                if n > 48 || data.len() < 3 + n {
                    return false;
                }
                self.work = vec![(le16(data, 0), data[3..3 + n].to_vec())];
                self.reading = false;
                self.batch = false;
            }
            _ => return false,
        }
        self.index = 0;
        self.status = Status {
            code: 1,
            id: self.work[0].0,
            ..Status::default()
        };
        self.start_pending = true;
        self.retries = 0;
        self.mismatches = 0;
        self.reunlocked = false;
        true
    }
    fn fail(&mut self, now: u32, code: u8, detail: u8) {
        self.status.code = code;
        self.status.phase = Phase::Failed;
        self.status.detail = detail;
        self.terminal = Some(now);
        self.start_pending = false;
        self.response = None;
    }
    fn done(&mut self, now: u32, readback: &[u8]) {
        self.status.code = if self.reading { 7 } else { 4 };
        self.status.phase = Phase::Done;
        self.status.detail = if self.reading { 100 } else { 0 };
        if self.reading {
            self.status.id = self.result.len() as u16;
        }
        self.status.readback = readback[..readback.len().min(8)].to_vec();
        self.terminal = Some(now);
    }
    fn request(&mut self, now: u32) -> Request {
        self.response = None;
        self.last_send = now;
        match self.status.phase {
            Phase::Unlock => Request {
                id: 225,
                payload: UNLOCK.to_vec(),
            },
            Phase::Write | Phase::BatchWrite => {
                let (id, d) = &self.work[self.index];
                let mut payload = id.to_le_bytes().to_vec();
                payload.extend_from_slice(d);
                Request { id: 220, payload }
            }
            Phase::Save => Request {
                id: 222,
                payload: vec![],
            },
            Phase::WaitReboot if self.require_unlock && !self.reunlocked => Request {
                id: 225,
                payload: UNLOCK.to_vec(),
            },
            _ => Request {
                id: 221,
                payload: self.work[self.index].0.to_le_bytes().to_vec(),
            },
        }
    }
    fn advance(&mut self, now: u32, verify: bool) -> Vec<Request> {
        self.index += 1;
        self.retries = 0;
        if self.index == self.work.len() {
            if self.reading {
                self.done(now, &[]);
                vec![]
            } else if verify {
                if self.mismatches > 0 {
                    self.fail(now, 0x83, self.mismatches.min(255) as u8);
                } else {
                    self.done(now, &[]);
                }
                vec![]
            } else {
                self.index = 0;
                self.status.phase = Phase::Save;
                vec![self.request(now)]
            }
        } else {
            self.status.detail = (self.index * 100 / self.work.len()) as u8;
            vec![self.request(now)]
        }
    }
    pub fn tick(&mut self, now: u32, state: State) -> Vec<Request> {
        if !self.active() {
            return vec![];
        }
        if state != State::Disarmed {
            self.fail(now, 0x81, 0);
            return vec![];
        }
        if self.start_pending {
            self.start_pending = false;
            self.status.phase = if self.require_unlock {
                Phase::Unlock
            } else if self.reading {
                Phase::Reading
            } else if self.batch {
                Phase::BatchWrite
            } else {
                Phase::Write
            };
            self.status.code = if self.reading { 6 } else { 2 };
            return vec![self.request(now)];
        }
        let response = self.response.take();
        let id = self.work[self.index].0;
        let timeout = if self.status.phase == Phase::Reading
            && [0x0140, 0x0141, 0x060e, 0x060f, 0x0610].contains(&id)
        {
            1200
        } else {
            200
        };
        let timed = now.wrapping_sub(self.last_send) > timeout;
        match self.status.phase {
            Phase::Unlock => {
                if response.is_some() {
                    self.retries = 0;
                    self.status.phase = if self.reading {
                        Phase::Reading
                    } else if self.batch {
                        Phase::BatchWrite
                    } else {
                        Phase::Write
                    };
                    return vec![self.request(now)];
                }
            }
            Phase::Reading => {
                if let Some((_, d)) = response {
                    let flag = if d.len() >= 3 { d[2] } else { 255 };
                    let data = if d.len() >= 3 {
                        &d[3..d.len().min(51)]
                    } else {
                        &[]
                    };
                    if self.result.len() + 4 + data.len() <= 3072 {
                        self.result.extend_from_slice(&id.to_le_bytes());
                        self.result.extend_from_slice(&[flag, data.len() as u8]);
                        self.result.extend_from_slice(data);
                    }
                    return self.advance(now, false);
                }
            }
            Phase::Write | Phase::BatchWrite => {
                if let Some((_, d)) = response {
                    if d.len() >= 3 && le16(&d, 0) == id && matches!(d[2], 0 | 6 | 7) {
                        self.retries = 0;
                        if self.batch {
                            return self.advance(now, false);
                        }
                        self.status.phase = Phase::Save;
                        return vec![self.request(now)];
                    } else if self.retries >= 8 {
                        self.fail(now, 0x85, d.get(2).copied().unwrap_or(255));
                        return vec![];
                    } else {
                        self.retries += 1;
                        return vec![self.request(now)];
                    }
                }
            }
            Phase::Save => {
                if let Some((_, d)) = response {
                    if d.first() == Some(&0) {
                        self.status.code = 3;
                        self.status.phase = Phase::WaitReboot;
                        self.phase_start = now;
                        self.last_probe = 0;
                        return vec![restart()];
                    } else {
                        self.fail(now, 0x80, d.first().copied().unwrap_or(255));
                        return vec![];
                    }
                }
            }
            Phase::WaitReboot => {
                let elapsed = now.wrapping_sub(self.phase_start);
                if elapsed < 5000 {
                    return vec![];
                }
                if self.require_unlock && !self.reunlocked {
                    if response.is_some() {
                        self.reunlocked = true;
                    }
                    if elapsed > 10000 {
                        self.fail(now, 0x82, 0);
                        return vec![];
                    }
                    if !self.reunlocked && now.wrapping_sub(self.last_send) > 500 {
                        return vec![self.request(now)];
                    }
                    if !self.reunlocked {
                        return vec![];
                    }
                }
                if self.batch {
                    self.index = 0;
                    self.mismatches = 0;
                    self.retries = 0;
                    self.status.phase = Phase::BatchVerify;
                    self.status.code = 2;
                    return vec![self.request(now)];
                }
                self.status.code = 2;
                self.status.phase = Phase::WaitReboot;
                if let Some((reply_id, d)) = response {
                    if reply_id == 221 && d.len() >= 3 && le16(&d, 0) == id && d[2] == 0 {
                        if d[3..].starts_with(&self.work[0].1) {
                            self.done(now, &d[3..]);
                        } else {
                            self.fail(now, 0x83, 0);
                        }
                        return vec![];
                    }
                }
                if elapsed > 14000 {
                    self.fail(now, 0x82, 0);
                    return vec![];
                }
                if now.wrapping_sub(self.last_probe) > 400 {
                    self.last_probe = now;
                    let r = self.request(now);
                    self.status.phase = Phase::WaitReboot;
                    return vec![r];
                }
                return vec![];
            }
            Phase::BatchVerify => {
                if let Some((_, d)) = response {
                    if d.len() < 3
                        || le16(&d, 0) != id
                        || d[2] != 0
                        || !d[3..].starts_with(&self.work[self.index].1)
                    {
                        self.mismatches += 1;
                    }
                    return self.advance(now, true);
                }
            }
            _ => {}
        }
        if timed {
            if self.retries < 8 {
                self.retries += 1;
                return vec![self.request(now)];
            }
            if self.status.phase == Phase::Reading {
                if self.result.len() + 4 <= 3072 {
                    self.result.extend_from_slice(&id.to_le_bytes());
                    self.result.extend_from_slice(&[255, 0]);
                }
                return self.advance(now, false);
            }
            if self.status.phase == Phase::BatchVerify {
                self.mismatches += 1;
                return self.advance(now, true);
            }
            self.fail(now, 0x82, 0);
        }
        vec![]
    }
}
