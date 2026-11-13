use controller_core::{Controller, Input, Role, State};
use controller_protocols::wireless::{FreshControl, RemotePublisher};
#[test]
fn stalled_remote_control_task_stops_rf_and_receiver_idles_without_waiting_for_remote_watchdog() {
    let mut tx = RemotePublisher::new(123);
    let mut receiver = Controller::new(Role::Receiver);
    for (now, state, raw) in [(500, 0, 0), (520, 1, 0), (540, 1, 4095), (560, 1, 4095)] {
        tx.enqueue(FreshControl {
            captured_at: now,
            throttle: raw,
            state,
            mode: 1,
        });
        let p = tx.take(now).unwrap();
        assert!(receiver.receive_control(
            now,
            p.session,
            p.sequence,
            p.throttle,
            State::from_byte(p.state).unwrap(),
            p.mode
        ));
        receiver.tick(Input {
            now_ms: now,
            ..Default::default()
        });
    }
    assert_eq!(receiver.state, State::Armed);
    // The worker keeps running, but its producer is stopped. No new packet can be manufactured.
    for now in (580..=720).step_by(20) {
        assert!(tx.take(now).is_none());
        let output = receiver.tick(Input {
            now_ms: now,
            ..Default::default()
        });
        if now <= 710 {
            assert_eq!(output.state, State::Armed);
        } else {
            assert_eq!(output.state, State::Disarmed);
            assert_eq!(output.pwm, 1000);
        }
    }
}
