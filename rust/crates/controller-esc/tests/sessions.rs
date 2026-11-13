use controller_core::State;
use controller_dronecan::Transfer;
use controller_esc::{
    firmware::Phase as FlashPhase, relay::Phase as RelayPhase, Esc, Flasher, Relay,
};
#[test]
fn telemetry_runtime_wrap_and_invalid_motor_are_preserved() {
    let mut e = Esc::new().unwrap();
    let mut b = vec![0; 33];
    b[28..30].copy_from_slice(&65000u16.to_le_bytes());
    b[26..28].copy_from_slice(&(-200i16).to_le_bytes());
    e.dispatch(
        Transfer {
            id: 213,
            kind: 0,
            source: 0x20,
            transfer_id: 1,
            data: b.clone(),
        },
        100,
    );
    e.tick(100, State::Disarmed);
    assert_eq!(e.telemetry.esc_status, 1);
    assert_eq!(e.telemetry.esc_temp_motor_dC, i16::MIN);
    b[28..30].copy_from_slice(&100u16.to_le_bytes());
    e.dispatch(
        Transfer {
            id: 213,
            kind: 0,
            source: 0x20,
            transfer_id: 2,
            data: b.clone(),
        },
        120,
    );
    assert_eq!(e.telemetry.esc_runtime_ms, 6360);
    e.tick(171, State::Disarmed);
    assert_eq!(e.telemetry.esc_status, 0);
    e.dispatch(
        Transfer {
            id: 213,
            kind: 0,
            source: 0x20,
            transfer_id: 3,
            data: b,
        },
        180,
    );
    e.tick(180, State::Disarmed);
    assert_eq!(
        e.telemetry.esc_status, 0,
        "unchanged ESC clock is stale despite repeated frames"
    );
}
#[test]
fn config_single_write_save_restart_verify_and_settle() {
    let mut r = Relay::default();
    assert!(r.command(0x10, &[0x80, 0, 2, 1, 0], false));
    let q = r.tick(500, State::Disarmed);
    assert_eq!(q[0].id, 225);
    r.response(225, &[1, 0, 0]);
    let q = r.tick(520, State::Disarmed);
    assert_eq!(q[0].id, 220);
    assert_eq!(q[0].payload, [0x80, 0, 1, 0]);
    r.response(220, &[0x80, 0, 0]);
    assert_eq!(r.tick(540, State::Disarmed)[0].id, 222);
    r.response(222, &[0]);
    assert_eq!(r.tick(560, State::Disarmed)[0].id, 5);
    assert!(r.tick(5559, State::Disarmed).is_empty());
    let q = r.tick(5560, State::Disarmed);
    assert_eq!(q[0].id, 225);
    r.response(225, &[1, 0, 0]);
    assert_eq!(r.tick(5580, State::Disarmed)[0].id, 221);
    assert_eq!(r.status.phase, RelayPhase::Verify);
    assert_eq!(r.status.encode()[3], 6);
    r.response(221, &[0x80, 0, 0, 1, 0]);
    r.tick(5600, State::Disarmed);
    assert_eq!(r.status.code, 4);
    assert_eq!(r.status.phase, RelayPhase::Done);
    assert!(!r.active());
    assert!(r.settle(14599));
    assert!(!r.settle(14600));
}
#[test]
fn armed_config_rejected_before_any_can_request() {
    let mut r = Relay::default();
    r.command(0x30, &[], false);
    assert!(r.tick(500, State::Armed).is_empty());
    assert_eq!(r.status.code, 0x81);
    assert!(!r.active());
}
#[test]
fn read_timeout_skips_with_sentinel_and_reads_full_catalog() {
    let mut r = Relay::default();
    r.require_unlock = false;
    r.command(0x30, &[], false);
    r.tick(500, State::Disarmed);
    for i in 1..=9 {
        r.tick(500 + i * 201, State::Disarmed);
    }
    assert_eq!(&r.result[..4], &[1, 0, 255, 0]);
    assert_eq!(r.status.phase, RelayPhase::Reading);
}
#[test]
fn firmware_receive_bounds_gate_hw_and_contiguous_progress() {
    let mut f = Flasher::default();
    assert!(!f.begin(1, 288, 2, State::Disarmed, false, 100));
    assert_eq!(f.code, 0x83);
    assert!(f.begin(1, 288, 1, State::Disarmed, false, 200));
    assert!(f.chunk(287, &[0; 2], 220).is_none());
    assert!(!f.end(230));
    let mut image = vec![0; 288];
    image[0] = 1;
    image[4] = 1;
    assert_eq!(f.chunk(0, &image, 240), Some(288));
    assert!(f.end(250));
    assert_eq!(f.tick(260, State::Disarmed)[0].id, 5);
    assert_eq!(f.tick(960, State::Disarmed)[0].id, 200);
    f.response(200, &[2]);
    assert_eq!(f.tick(980, State::Disarmed)[0].id, 201);
    f.response(201, &[0]);
    let q = f.tick(1000, State::Disarmed);
    assert_eq!(q[0].id, 202);
    assert_eq!(q[0].payload.len(), 258);
    f.response(202, &[0, 1, 0]);
    assert_eq!(f.tick(1020, State::Disarmed)[0].id, 203);
    f.response(203, &[0]);
    f.tick(1040, State::Disarmed);
    assert_eq!(f.phase, FlashPhase::Done);
    assert_eq!(f.progress, 1000);
}
#[test]
fn firmware_receive_timeout_and_arm_abort() {
    let mut f = Flasher::default();
    assert!(f.begin(1, 288, 1, State::Disarmed, false, 100));
    f.tick(15101, State::Disarmed);
    assert_eq!(f.code, 0x82);
    assert!(!f.active());
    assert!(f.begin(1, 32, 1, State::Disarmed, false, 20000));
    let mut image = [0; 32];
    image[0] = 1;
    f.chunk(0, &image, 20020);
    f.end(20040);
    f.tick(20060, State::Armed);
    assert_eq!(f.code, 0x81);
}

#[test]
fn firmware_staging_busy_retries_without_can_or_session_failure() {
    use controller_esc::firmware::{ImageStorage, MemoryImage};
    #[derive(Debug)]
    struct BusyOnce {
        memory: MemoryImage,
        busy: bool,
        tried: bool,
    }
    impl ImageStorage for BusyOnce {
        fn reserve(&mut self, n: usize) -> bool {
            self.memory.reserve(n)
        }
        fn len(&self) -> usize {
            self.memory.len()
        }
        fn write(&mut self, o: usize, d: &[u8]) -> bool {
            self.memory.write(o, d)
        }
        fn read(&mut self, o: usize, d: &mut [u8]) -> bool {
            if !self.tried {
                self.tried = true;
                self.busy = true;
                false
            } else {
                self.busy = false;
                self.memory.read(o, d)
            }
        }
        fn read_retryable(&self) -> bool {
            self.busy
        }
        fn seal(&mut self) -> bool {
            self.memory.seal()
        }
        fn clear(&mut self) {
            self.memory.clear();
        }
    }
    let mut f = Flasher::with_storage(Box::new(BusyOnce {
        memory: MemoryImage::default(),
        busy: false,
        tried: false,
    }));
    assert!(f.begin(1, 32, 1, State::Disarmed, false, 100));
    let mut image = [0u8; 32];
    image[0] = 1;
    f.chunk(0, &image, 110);
    assert!(!f.end(120));
    assert_eq!(f.tick(125, State::Disarmed)[0].id, 5);
    assert!(f.tick(130, State::Disarmed).is_empty());
    assert_eq!(f.tick(830, State::Disarmed)[0].id, 200);
    f.response(200, &[2]);
    assert_eq!(f.tick(850, State::Disarmed)[0].id, 201);
    assert_eq!(f.phase, FlashPhase::Starting);
}

#[test]
fn wrong_image_hardware_rejected_before_esc_restart() {
    let mut f = Flasher::default();
    assert!(f.begin(3, 32, 3, State::Disarmed, false, 100));
    let mut image = [0; 32];
    image[0] = 4;
    f.chunk(0, &image, 110);
    assert!(!f.end(120));
    assert_eq!(f.code, 0x83);
    assert!(!f.active());
    assert!(f.tick(130, State::Disarmed).is_empty());
}

#[test]
fn read_all_reports_incremental_percentage_on_wire() {
    use controller_esc::relay::PARAM_IDS;
    let mut r = Relay::default();
    r.require_unlock = false;
    r.command(0x30, &[], false);
    r.tick(100, State::Disarmed);
    for (index, id) in PARAM_IDS.iter().take(8).enumerate() {
        let bytes = id.to_le_bytes();
        r.response(221, &[bytes[0], bytes[1], 0]);
        r.tick(110 + index as u32 * 10, State::Disarmed);
        assert_eq!(r.status.phase, RelayPhase::Reading);
        assert_eq!(
            r.status.encode()[4],
            ((index + 1) * 100 / PARAM_IDS.len()) as u8
        );
    }
    assert!(r.status.detail > 0);
}

#[test]
fn esc_hardware_change_during_upload_is_rejected_before_restart() {
    let mut f = Flasher::default();
    assert!(!f.end_for_hardware(0, 3));
    assert_eq!(f.code, 0);
    assert!(f.begin(3, 32, 3, State::Disarmed, false, 100));
    let mut image = [0; 32];
    image[0] = 3;
    f.chunk(0, &image, 110);
    assert!(!f.end_for_hardware(120, 4));
    assert_eq!(f.code, 0x83);
    assert!(f.tick(130, State::Disarmed).is_empty());
}
