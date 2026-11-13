//! Independent pinned-C encoders versus real Rust requests and CAN reassembly.
use controller_core::State;
use controller_dronecan::DroneCan;
use controller_esc::{signature, Esc, Flasher, Request};

unsafe extern "C" {
    fn eppg_test_codec(fixture: u32, out: *mut u8) -> u32;
}
fn compare(fixture: u32, request: Request) {
    let mut expected = [0u8; 300];
    // All fixture messages have a maximum encoded length of 258 bytes.
    let len = unsafe { eppg_test_codec(fixture, expected.as_mut_ptr()) } as usize;
    assert_eq!(request.payload, expected[..len], "fixture {fixture}");
}
fn receive_request(esc: &mut Esc, id: u16) -> Request {
    let mut peer = DroneCan::new(0x20).unwrap();
    assert!(peer.register(id, 1, 1, signature(id).unwrap()));
    let mut time = 100_000;
    while let Some(frame) = esc.transport.peek_frame() {
        assert!(peer.receive(frame, time) >= 0);
        esc.transport.pop_frame();
        time += 130;
    }
    let transfer = peer.take_transfer().expect("reassembled request");
    assert_eq!(transfer.id, id);
    assert!(peer.take_transfer().is_none());
    Request {
        id,
        payload: transfer.data,
    }
}
#[test]
fn motor_tones_and_every_status_led_pattern_match_vendor_tao_encoders() {
    for arm in [true, false] {
        let mut esc = Esc::new().unwrap();
        esc.telemetry.esc_status = 1;
        esc.motor_tone(arm);
        compare(u32::from(!arm), receive_request(&mut esc, 214));
    }
    for color in 1..=3 {
        for armed in [false, true] {
            let mut esc = Esc::new().unwrap();
            esc.telemetry.esc_status = 1;
            esc.status_light(
                if armed { State::Armed } else { State::Disarmed },
                color == 3,
                color == 1,
            );
            compare(
                2 + (color - 1) * 2 + u32::from(armed),
                receive_request(&mut esc, 218),
            );
        }
    }
    let mut esc = Esc::new().unwrap();
    esc.status_light(State::Disarmed, false, false);
    compare(8, receive_request(&mut esc, 218));
}
#[test]
fn firmware_requests_match_vendor_restart_header_and_fixed_chunk_encoders() {
    let mut flash = Flasher::default();
    assert!(flash.begin(0x1234, 288, 0x1234, State::Disarmed, false, 100));
    let mut image = [0u8; 288];
    image[..2].copy_from_slice(&0x1234u16.to_le_bytes());
    image[4..6].copy_from_slice(&0x5678u16.to_le_bytes());
    image[6..8].copy_from_slice(&0x9abcu16.to_le_bytes());
    for (i, b) in image[32..].iter_mut().enumerate() {
        *b = i as u8;
    }
    assert_eq!(flash.chunk(0, &image, 110), Some(288));
    assert!(flash.end(120));
    compare(9, flash.tick(130, State::Disarmed).remove(0));
    assert_eq!(flash.tick(830, State::Disarmed)[0].id, 200);
    flash.response(200, &[2]);
    compare(10, flash.tick(850, State::Disarmed).remove(0));
    flash.response(201, &[0]);
    compare(11, flash.tick(870, State::Disarmed).remove(0));
}
