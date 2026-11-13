//! ESP-NOW v1 application protocol, independent of radio security.
pub const MAGIC: u16 = 0x5050;
pub const VERSION: u8 = 1;
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for b in data {
        crc ^= *b as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320u32 & 0u32.wrapping_sub(crc & 1));
        }
    }
    !crc
}
pub fn sequence_newer(candidate: u32, previous: u32) -> bool {
    candidate.wrapping_sub(previous) as i32 > 0
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Control {
    pub session: u32,
    pub sequence: u32,
    pub uptime: u32,
    pub throttle: u16,
    pub state: u8,
    pub mode: u8,
    pub flags: u8,
    pub reserved: u8,
}
impl Control {
    pub fn encode(self) -> [u8; 26] {
        let mut b = [0; 26];
        b[..2].copy_from_slice(&MAGIC.to_le_bytes());
        b[2] = VERSION;
        b[3] = 1;
        b[4..8].copy_from_slice(&self.session.to_le_bytes());
        b[8..12].copy_from_slice(&self.sequence.to_le_bytes());
        b[12..16].copy_from_slice(&self.uptime.to_le_bytes());
        b[16..18].copy_from_slice(&self.throttle.to_le_bytes());
        b[18] = self.state;
        b[19] = self.mode;
        b[20] = self.flags;
        b[21] = self.reserved;
        let crc = crc32(&b[..22]);
        b[22..].copy_from_slice(&crc.to_le_bytes());
        b
    }
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() != 26 || !header(b, 1) || b[18] > 2 || b[19] > 1 {
            return None;
        }
        let throttle = u16::from_le_bytes(b[16..18].try_into().ok()?);
        if throttle > 4095 {
            return None;
        }
        Some(Self {
            session: u32::from_le_bytes(b[4..8].try_into().ok()?),
            sequence: u32::from_le_bytes(b[8..12].try_into().ok()?),
            uptime: u32::from_le_bytes(b[12..16].try_into().ok()?),
            throttle,
            state: b[18],
            mode: b[19],
            flags: b[20],
            reserved: b[21],
        })
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ack {
    pub session: u32,
    pub sequence: u32,
    pub uptime: u32,
    pub state: u8,
    pub flags: u8,
    pub age: u16,
}
impl Ack {
    pub const LINK_READY: u8 = 1;
    pub const FAILSAFE: u8 = 2;
    pub const ACCEPTED: u8 = 4;
    pub fn encode(self) -> [u8; 24] {
        let mut b = [0; 24];
        b[..2].copy_from_slice(&MAGIC.to_le_bytes());
        b[2] = VERSION;
        b[3] = 2;
        b[4..8].copy_from_slice(&self.session.to_le_bytes());
        b[8..12].copy_from_slice(&self.sequence.to_le_bytes());
        b[12..16].copy_from_slice(&self.uptime.to_le_bytes());
        b[16] = self.state;
        b[17] = self.flags;
        b[18..20].copy_from_slice(&self.age.to_le_bytes());
        let crc = crc32(&b[..20]);
        b[20..].copy_from_slice(&crc.to_le_bytes());
        b
    }
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() != 24 || !header(b, 2) || b[16] > 2 {
            return None;
        }
        Some(Self {
            session: u32::from_le_bytes(b[4..8].try_into().ok()?),
            sequence: u32::from_le_bytes(b[8..12].try_into().ok()?),
            uptime: u32::from_le_bytes(b[12..16].try_into().ok()?),
            state: b[16],
            flags: b[17],
            age: u16::from_le_bytes(b[18..20].try_into().ok()?),
        })
    }
}
fn header(b: &[u8], kind: u8) -> bool {
    b.len() >= 8
        && b[..2] == MAGIC.to_le_bytes()
        && b[2] == VERSION
        && b[3] == kind
        && crc32(&b[..b.len() - 4])
            == u32::from_le_bytes(b[b.len() - 4..].try_into().unwrap_or([0; 4]))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn crc_known() {
        assert_eq!(crc32(b"123456789"), 0xcbf43926);
    }
    #[test]
    fn control_roundtrip() {
        let c = Control {
            session: 123,
            sequence: 456,
            uptime: 789,
            throttle: 2048,
            state: 1,
            mode: 1,
            ..Control::default()
        };
        assert_eq!(Control::decode(&c.encode()), Some(c));
        let mut bad = c.encode();
        bad[9] ^= 1;
        assert!(Control::decode(&bad).is_none());
    }
    #[test]
    fn sequence_wrap() {
        assert!(sequence_newer(0, u32::MAX));
        assert!(!sequence_newer(10, 10));
        assert!(!sequence_newer(9, 10));
    }
    #[test]
    fn ack_roundtrip() {
        let a = Ack {
            session: 123,
            sequence: 456,
            uptime: 789,
            state: 2,
            flags: 7,
            age: 150,
        };
        assert_eq!(Ack::decode(&a.encode()), Some(a));
    }
}

/// The radio worker may publish only a newly supplied control-loop sample.
/// A stalled producer must stop commands so the receiver's 150 ms failsafe works.
#[derive(Clone, Copy, Debug)]
pub struct FreshControl {
    pub captured_at: u32,
    pub throttle: u16,
    pub state: u8,
    pub mode: u8,
}
pub struct RemotePublisher {
    session: u32,
    sequence: u32,
    pending: Option<FreshControl>,
}
impl RemotePublisher {
    pub fn new(session: u32) -> Self {
        Self {
            session,
            sequence: 0,
            pending: None,
        }
    }
    pub fn enqueue(&mut self, sample: FreshControl) {
        self.pending = Some(sample);
    }
    pub fn take(&mut self, now: u32) -> Option<Control> {
        let sample = self.pending.take()?;
        if now.wrapping_sub(sample.captured_at) > 40 {
            return None;
        }
        self.sequence = self.sequence.wrapping_add(1);
        Some(Control {
            session: self.session,
            sequence: self.sequence,
            uptime: now,
            throttle: sample.throttle,
            state: sample.state,
            mode: sample.mode,
            flags: 0,
            reserved: 0,
        })
    }
}
#[cfg(test)]
mod publisher_tests {
    use super::*;
    #[test]
    fn stopped_control_producer_cannot_be_refreshed_by_radio_worker() {
        let mut tx = RemotePublisher::new(1);
        for at in (500..=600).step_by(20) {
            tx.enqueue(FreshControl {
                captured_at: at,
                throttle: 4095,
                state: 1,
                mode: 1,
            });
            assert!(tx.take(at).is_some());
            assert!(tx.take(at + 1).is_none());
        }
        for now in (620..=1000).step_by(20) {
            assert!(
                tx.take(now).is_none(),
                "radio must not manufacture new sequence numbers from a held sample"
            );
        }
        tx.enqueue(FreshControl {
            captured_at: 600,
            throttle: 4095,
            state: 1,
            mode: 1,
        });
        assert!(tx.take(641).is_none());
    }
    #[test]
    fn only_latest_sample_and_uptime_wrap_are_supported() {
        let mut tx = RemotePublisher::new(123);
        tx.enqueue(FreshControl {
            captured_at: u32::MAX - 10,
            throttle: 100,
            state: 0,
            mode: 0,
        });
        let packet = tx.take(10).unwrap();
        assert_eq!(packet.sequence, 1);
        assert_eq!(packet.throttle, 100);
        tx.enqueue(FreshControl {
            captured_at: 20,
            throttle: 200,
            state: 1,
            mode: 0,
        });
        tx.enqueue(FreshControl {
            captured_at: 40,
            throttle: 300,
            state: 1,
            mode: 1,
        });
        assert_eq!(tx.take(40).unwrap().throttle, 300);
    }
}

/// DroneCAN v0 uses only extended data frames. Transport must not invent EFF.
pub fn dronecan_frame_allowed(extended: bool, remote: bool, len: u8) -> bool {
    extended && !remote && len <= 8
}
#[cfg(test)]
mod frame_tests {
    use super::*;
    #[test]
    fn standard_rtr_and_oversized_frames_cannot_masquerade_as_dronecan() {
        assert!(dronecan_frame_allowed(true, false, 8));
        assert!(!dronecan_frame_allowed(false, false, 8));
        assert!(!dronecan_frame_allowed(true, true, 8));
        assert!(!dronecan_frame_allowed(true, false, 9));
    }
}
