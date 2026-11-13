use controller_core::{Controller, Effects, Input, Role, State};
fn tick(c: &mut Controller, t: u32, raw: u16) -> controller_core::Output {
    c.tick(Input {
        now_ms: t,
        raw,
        button_down: false,
        maintenance: false,
    })
}
#[test]
fn boot_idle_cooldown_and_exact_integer_arm_threshold() {
    let mut c = Controller::default();
    assert_eq!(tick(&mut c, 0, 4095).pwm, 1000);
    c.toggle_arm(499, 0, false);
    assert_eq!(c.state, State::Disarmed);
    c.toggle_arm(500, 204, false);
    assert_eq!(c.state, State::Disarmed);
    assert!(tick(&mut c, 510, 204).effects.contains(Effects::ARM_FAIL));
    c.toggle_arm(520, 203, false);
    assert_eq!(c.state, State::Armed);
    assert_eq!(tick(&mut c, 540, 4095).pwm, 1043);
}
#[test]
fn maintenance_prevents_arm_but_never_disarm() {
    let mut c = Controller::default();
    c.toggle_arm(500, 0, true);
    assert_eq!(c.state, State::Disarmed);
    c.toggle_arm(520, 0, false);
    assert_eq!(c.state, State::Armed);
    c.toggle_arm(540, 4095, true);
    assert_eq!(c.state, State::Disarmed);
    assert_eq!(tick(&mut c, 560, 4095).pwm, 1000);
}
#[test]
fn cruise_holds_flown_command_on_override_cycle_and_refills_filter() {
    let mut c = Controller::default();
    c.mode = 1;
    c.toggle_arm(500, 0, false);
    for t in (520..1000).step_by(20) {
        tick(&mut c, t, 2000);
    }
    let flown = c.pwm;
    c.toggle_cruise(1000, 2000, false);
    assert_eq!(c.state, State::Cruising);
    let cruise = c.cruise_pwm;
    assert_eq!(cruise, flown);
    assert_eq!(tick(&mut c, 3000, 1600).state, State::Cruising);
    let o = tick(&mut c, 3020, 1600);
    assert_eq!(o.state, State::Armed);
    assert_eq!(o.pwm, cruise);
    assert!(o.effects.contains(Effects::CRUISE_END));
}
#[test]
fn cruise_external_value_uses_shared_seventy_percent_cap() {
    let mut c = Controller::default();
    assert!(!c.set_cruise_pwm(1500));
    c.toggle_arm(500, 0, false);
    c.toggle_cruise(600, 1500, false);
    assert!(!c.set_cruise_pwm(1800));
    assert!(c.set_cruise_pwm(1680));
    assert!(!c.set_cruise_pwm(1681));
    assert!(!c.set_cruise_pwm(1000));
    assert_eq!(tick(&mut c, 620, 0).pwm, 1680);
}
#[test]
fn receiver_fresh_disarmed_handshake_sequence_rollover_and_loss_latch() {
    let mut c = Controller::new(Role::Receiver);
    assert!(!c.receive_control(500, 1, 1, 0, State::Armed, 0));
    assert!(c.receive_control(500, 1, u32::MAX, 0, State::Disarmed, 0));
    tick(&mut c, 500, 0);
    assert!(c.receive_control(520, 1, 0, 0, State::Armed, 0));
    assert_eq!(tick(&mut c, 520, 0).state, State::Armed);
    assert!(!c.receive_control(530, 1, 0, 0, State::Disarmed, 0));
    assert_eq!(tick(&mut c, 670, 0).state, State::Armed);
    let o = tick(&mut c, 680, 0);
    assert_eq!(o.state, State::Disarmed);
    assert_eq!(o.pwm, 1000);
    assert!(c.failsafe);
    assert!(c.receive_control(700, 1, 1, 0, State::Armed, 0));
    assert_eq!(tick(&mut c, 700, 0).state, State::Disarmed);
    assert!(c.receive_control(720, 1, 2, 0, State::Disarmed, 0));
    tick(&mut c, 720, 0);
    assert!(c.receive_control(1220, 1, 3, 0, State::Armed, 0));
    assert_eq!(tick(&mut c, 1220, 0).state, State::Armed);
}
#[test]
fn remote_receiver_ack_is_authoritative_and_status_times_out() {
    let mut c = Controller::new(Role::Remote);
    c.toggle_arm(500, 204, false);
    assert_eq!(c.desired_state, State::Armed);
    c.receive_ack(520, State::Armed, false);
    assert_eq!(tick(&mut c, 820, 1000).state, State::Armed);
    let o = tick(&mut c, 830, 1000);
    assert_eq!(o.state, State::Disarmed);
    assert_eq!(o.desired_state, State::Disarmed);
    assert!(!o.send_throttle);
}
#[test]
fn click_then_hold_and_disarmed_long_hold_gestures() {
    let mut c = Controller::default();
    let mut step = |t, down| {
        c.tick(Input {
            now_ms: t,
            raw: 0,
            button_down: down,
            maintenance: false,
        })
    };
    step(100, false);
    step(200, true);
    step(300, false);
    step(400, true);
    step(2400, true);
    assert_eq!(c.state, State::Armed);
    let mut c = Controller::default();
    c.tick(Input {
        now_ms: 100,
        button_down: true,
        ..Default::default()
    });
    let o = c.tick(Input {
        now_ms: 3200,
        button_down: false,
        ..Default::default()
    });
    assert_eq!(c.mode, 1);
    assert!(o.effects.contains(Effects::PERSIST));
    let mut c = Controller::default();
    c.tick(Input {
        now_ms: 100,
        button_down: true,
        ..Default::default()
    });
    assert!(c
        .tick(Input {
            now_ms: 10100,
            button_down: true,
            ..Default::default()
        })
        .effects
        .contains(Effects::PAIRING));
    assert!(!c
        .tick(Input {
            now_ms: 10200,
            button_down: true,
            ..Default::default()
        })
        .effects
        .contains(Effects::PAIRING));
}
#[test]
fn uptime_wrap_ramp_and_remote_sequence_do_not_rearm_on_reboot() {
    let mut c = Controller::new(Role::Receiver);
    assert!(c.receive_control(u32::MAX - 30, 7, 100, 0, State::Disarmed, 0));
    tick(&mut c, u32::MAX - 30, 0);
    assert!(c.receive_control(u32::MAX - 10, 7, 101, 0, State::Armed, 0));
    assert_eq!(tick(&mut c, u32::MAX - 10, 0).state, State::Armed);
    assert!(!c.receive_control(10, 8, 1, 4095, State::Armed, 1));
    assert_eq!(tick(&mut c, 10, 0).state, State::Armed);
    assert_eq!(tick(&mut c, 150, 0).state, State::Disarmed);
}

#[test]
fn both_modes_refuse_cruise_above_shared_output_limit_and_hold_during_ramp() {
    for mode in [0, 1] {
        let mut c = Controller::default();
        c.mode = mode;
        c.toggle_arm(500, 0, false);
        tick(&mut c, 520, 4095);
        let flown = c.pwm;
        // High pot target alone does not reject an output still below the cap.
        c.toggle_cruise(530, 4095, false);
        assert_eq!(c.state, State::Cruising);
        assert_eq!(tick(&mut c, 540, 4095).pwm, flown);
        c.toggle_cruise(550, 4095, false);
        for t in (560..4000).step_by(20) {
            tick(&mut c, t, 4095);
        }
        assert_eq!(c.pwm, if mode == 0 { 1770 } else { 1950 });
        c.toggle_cruise(4000, 4095, false);
        assert_eq!(c.state, State::Armed);
        assert!(tick(&mut c, 4020, 4095)
            .effects
            .contains(Effects::CRUISE_FAIL));
    }
}

#[test]
fn wireless_cruise_uses_mode_output_cap_and_receiver_command_is_authoritative() {
    let mut remote = Controller::new(Role::Remote);
    remote.mode = 0;
    remote.receive_ack(500, State::Armed, false);
    remote.toggle_cruise(520, 3500, false);
    assert_eq!(remote.desired_state, State::Cruising);
    remote.desired_state = State::Armed;
    remote.toggle_cruise(540, 4095, false);
    assert_eq!(remote.desired_state, State::Armed);

    let mut receiver = Controller::new(Role::Receiver);
    assert!(receiver.receive_control(500, 1, 1, 0, State::Disarmed, 0));
    tick(&mut receiver, 500, 0);
    assert!(receiver.receive_control(1000, 1, 2, 0, State::Armed, 0));
    tick(&mut receiver, 1000, 0);
    let mut seq = 3;
    for now in (1020..4000).step_by(20) {
        assert!(receiver.receive_control(now, 1, seq, 4095, State::Armed, 0));
        tick(&mut receiver, now, 0);
        seq += 1;
    }
    assert_eq!(receiver.pwm, 1770);
    assert!(receiver.receive_control(4000, 1, seq, 4095, State::Cruising, 0));
    let output = tick(&mut receiver, 4000, 0);
    assert_eq!(output.state, State::Armed);
    assert!(output.effects.contains(Effects::CRUISE_FAIL));
}
