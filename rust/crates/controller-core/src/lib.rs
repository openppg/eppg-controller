//! Deterministic SP140 controller. No peripheral, allocation or scheduler dependency.
#![no_std]
#![forbid(unsafe_code)]
pub mod monitor;
pub mod settings;
pub mod throttle;
pub use settings::Settings;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum State {
    #[default]
    Disarmed = 0,
    Armed = 1,
    Cruising = 2,
}
impl State {
    pub fn from_byte(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Disarmed),
            1 => Some(Self::Armed),
            2 => Some(Self::Cruising),
            _ => None,
        }
    }
}
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Role {
    #[default]
    Standalone = 0,
    Remote = 1,
    Receiver = 2,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Effects(pub u32);
impl Effects {
    pub const ARM: u32 = 1;
    pub const DISARM: u32 = 2;
    pub const CRUISE_START: u32 = 4;
    pub const CRUISE_END: u32 = 8;
    pub const ARM_FAIL: u32 = 16;
    pub const CRUISE_FAIL: u32 = 32;
    pub const PAIRING: u32 = 64;
    pub const MODE_CHANGED: u32 = 128;
    pub const PERSIST: u32 = 256;
    pub const ZERO_ALTITUDE: u32 = 512;
    pub fn contains(self, effect: u32) -> bool {
        self.0 & effect != 0
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Input {
    pub now_ms: u32,
    pub raw: u16,
    pub button_down: bool,
    pub maintenance: bool,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    pub state: State,
    pub desired_state: State,
    pub pwm: u16,
    pub send_throttle: bool,
    pub control_due: bool,
    pub effects: Effects,
}
#[derive(Clone, Copy, Debug)]
struct Button {
    last_down: bool,
    pressed: bool,
    last_edge: u32,
    press_at: u32,
    release_at: u32,
    sequence: bool,
    pairing_done: bool,
}
impl Button {
    const fn new() -> Self {
        Self {
            last_down: false,
            pressed: false,
            last_edge: 0,
            press_at: 0,
            release_at: 0,
            sequence: false,
            pairing_done: false,
        }
    }
}
/// State is exclusively owned by the control task. Other tasks enqueue typed requests.
#[derive(Clone, Debug)]
pub struct Controller {
    pub state: State,
    pub desired_state: State,
    pub role: Role,
    pub mode: u8,
    pub pwm: u16,
    pub armed_minutes: u16,
    pub failsafe: bool,
    pub cruise_pwm: u16,
    pub cruise_raw: u16,
    pub cruise_at: u32,
    pub last_disarm: u32,
    pub armed_at: u32,
    filter: throttle::Filter,
    previous_pwm: u16,
    last_control: Option<u32>,
    button: Button,
    effects: Effects,
    remote: Option<(u32, u32, u16, State, u8, u32)>,
    active_session: u32,
    previous_requested: State,
    ready_for_arm: bool,
    last_ack: Option<u32>,
}
impl Controller {
    pub const fn new(role: Role) -> Self {
        Self {
            state: State::Disarmed,
            desired_state: State::Disarmed,
            role,
            mode: 0,
            pwm: 1000,
            armed_minutes: 0,
            failsafe: true,
            cruise_pwm: 1035,
            cruise_raw: 0,
            cruise_at: 0,
            last_disarm: 0,
            armed_at: 0,
            filter: throttle::Filter::new(),
            previous_pwm: 1035,
            last_control: None,
            button: Button::new(),
            effects: Effects(0),
            remote: None,
            active_session: 0,
            previous_requested: State::Disarmed,
            ready_for_arm: false,
            last_ack: None,
        }
    }
    pub fn set_role(&mut self, role: Role) {
        *self = Self::new(role);
    }
    fn effect(&mut self, v: u32) {
        self.effects.0 |= v;
    }
    pub fn set_state(&mut self, now: u32, state: State, raw: u16) {
        let old = self.state;
        if old == state {
            return;
        }
        self.state = state;
        match state {
            State::Disarmed => {
                let seconds = now.wrapping_sub(self.armed_at) / 1000;
                // C++ round(armedSecs / 60) uses integer division before round.
                self.armed_minutes = self
                    .armed_minutes
                    .saturating_add((seconds / 60).min(65535) as u16);
                self.last_disarm = now;
                self.previous_pwm = 1035;
                self.filter.clear();
                self.pwm = 1000;
                self.effect(Effects::DISARM | Effects::PERSIST);
            }
            State::Armed if old == State::Disarmed => {
                self.armed_at = now;
                self.effect(Effects::ARM | Effects::ZERO_ALTITUDE);
            }
            State::Armed => {
                self.filter.reset(throttle::map_mode(raw, self.mode));
                self.cruise_raw = 0;
                self.effect(Effects::CRUISE_END);
            }
            State::Cruising => {
                self.cruise_raw = raw;
                self.cruise_at = now;
                // Capture the commanded output, matching master's throttle-owner latch.
                self.cruise_pwm = self.previous_pwm.min(throttle::CRUISE_MAX);
                self.effect(Effects::CRUISE_START);
            }
        }
    }
    pub fn toggle_arm(&mut self, now: u32, raw: u16, maintenance: bool) {
        if self.role == Role::Remote {
            if self.state == State::Disarmed && self.desired_state == State::Disarmed {
                if !maintenance {
                    if raw < 205 {
                        self.desired_state = State::Armed;
                    } else {
                        self.effect(Effects::ARM_FAIL);
                    }
                }
            } else {
                self.desired_state = State::Disarmed;
            }
            return;
        }
        if self.state == State::Disarmed {
            if !maintenance && now.wrapping_sub(self.last_disarm) >= 500 {
                if raw < 204 {
                    self.set_state(now, State::Armed, raw);
                } else {
                    self.effect(Effects::ARM_FAIL);
                }
            }
        } else {
            self.set_state(now, State::Disarmed, raw);
        }
    }
    pub fn toggle_cruise(&mut self, now: u32, raw: u16, maintenance: bool) {
        if self.state == State::Armed {
            if !maintenance {
                let engaged = raw >= if self.role == Role::Remote { 205 } else { 204 };
                let output = if self.role == Role::Remote {
                    throttle::map_mode(raw, self.mode)
                } else {
                    self.previous_pwm
                };
                if engaged && throttle::cruise_output_allowed(output) {
                    if self.role == Role::Remote {
                        self.desired_state = State::Cruising;
                    } else {
                        self.set_state(now, State::Cruising, raw);
                    }
                } else {
                    self.effect(Effects::CRUISE_FAIL);
                }
            }
        } else if self.state == State::Cruising {
            if self.role == Role::Remote {
                self.desired_state = State::Armed;
            } else {
                self.set_state(now, State::Armed, raw);
            }
        }
    }
    pub fn set_cruise_pwm(&mut self, pwm: u16) -> bool {
        if self.state == State::Cruising && (1035..=throttle::CRUISE_MAX).contains(&pwm) {
            self.cruise_pwm = pwm;
            true
        } else {
            false
        }
    }
    pub fn receive_control(
        &mut self,
        now: u32,
        session: u32,
        seq: u32,
        raw: u16,
        state: State,
        mode: u8,
    ) -> bool {
        if self.role != Role::Receiver || raw > 4095 || mode > 1 {
            return false;
        }
        if let Some((old_session, old_seq, ..)) = self.remote {
            if session == old_session {
                if (seq.wrapping_sub(old_seq) as i32) <= 0 {
                    return false;
                }
            } else if state != State::Disarmed {
                return false;
            }
        } else if state != State::Disarmed {
            return false;
        }
        if self.remote.map(|r| r.0) != Some(session) {
            self.failsafe = true;
        }
        self.remote = Some((session, seq, raw, state, mode, now));
        true
    }
    pub fn receive_ack(&mut self, now: u32, actual: State, failsafe: bool) {
        if self.role != Role::Remote {
            return;
        }
        self.last_ack = Some(now);
        self.desired_state = if failsafe { State::Disarmed } else { actual };
        self.set_state(now, actual, 0);
    }
    pub fn control_age(&self, now: u32) -> Option<u32> {
        self.remote.map(|r| now.wrapping_sub(r.5))
    }
    pub fn accepted_control(&self) -> Option<(u32, u32)> {
        self.remote.map(|r| (r.0, r.1))
    }
    fn receiver(&mut self, now: u32, maintenance: bool) -> u16 {
        let Some((session, _seq, raw, requested, mode, at)) = self.remote else {
            self.ready_for_arm = false;
            self.failsafe = true;
            self.set_state(now, State::Disarmed, 0);
            return 0;
        };
        if now.wrapping_sub(at) > 150 {
            self.ready_for_arm = false;
            self.failsafe = true;
            self.set_state(now, State::Disarmed, 0);
            return 0;
        }
        if self.active_session != session {
            self.active_session = session;
            self.previous_requested = State::Disarmed;
            self.ready_for_arm = false;
        }
        self.mode = mode;
        if requested == State::Disarmed {
            self.ready_for_arm = true;
            self.previous_requested = State::Disarmed;
            self.failsafe = false;
            self.set_state(now, State::Disarmed, raw);
            return raw;
        }
        if self.ready_for_arm && requested != self.previous_requested {
            if requested == State::Armed {
                if self.state == State::Disarmed {
                    self.toggle_arm(now, raw, maintenance);
                } else if self.state == State::Cruising {
                    self.toggle_cruise(now, raw, maintenance);
                }
            } else if requested == State::Cruising && self.state == State::Armed {
                self.toggle_cruise(now, raw, maintenance);
            }
            self.previous_requested = requested;
        }
        raw
    }
    fn button(&mut self, input: Input, raw: u16) {
        let now = input.now_ms;
        if now.wrapping_sub(self.button.last_edge) <= 50 {
            return;
        }
        if input.button_down != self.button.last_down {
            self.button.last_edge = now;
            self.button.last_down = input.button_down;
            if input.button_down {
                self.button.pressed = true;
                self.button.pairing_done = false;
                self.button.press_at = now;
            } else {
                self.button.pressed = false;
                self.button.release_at = now;
                let held = now.wrapping_sub(self.button.press_at);
                if self.state == State::Disarmed
                    && (3000..10000).contains(&held)
                    && !self.button.pairing_done
                {
                    self.mode = if self.mode == 0 { 1 } else { 0 };
                    self.effect(Effects::MODE_CHANGED | Effects::PERSIST);
                    return;
                }
                if !self.button.sequence && held < 500 {
                    self.button.sequence = true;
                }
            }
        }
        if self.button.sequence {
            if !self.button.pressed && now.wrapping_sub(self.button.release_at) > 1500 {
                self.button.sequence = false;
            }
            if self.button.pressed && now.wrapping_sub(self.button.press_at) >= 2000 {
                self.toggle_arm(now, raw, input.maintenance);
                self.button.sequence = false;
                self.button.pressed = false;
                self.button.press_at = now;
            }
        } else if self.button.pressed {
            let held = now.wrapping_sub(self.button.press_at);
            if self.state == State::Disarmed && held >= 10000 && !self.button.pairing_done {
                self.effect(Effects::PAIRING);
                self.button.pairing_done = true;
            }
            if self.state != State::Disarmed && held >= 2000 {
                self.toggle_cruise(now, raw, input.maintenance);
                self.button.pressed = false;
                self.button.press_at = now;
            }
        }
    }
    pub fn tick(&mut self, input: Input) -> Output {
        let now = input.now_ms;
        let raw = if self.role == Role::Receiver {
            self.receiver(now, input.maintenance)
        } else {
            input.raw.min(4095)
        };
        if self.role == Role::Remote
            && self
                .last_ack
                .map(|at| now.wrapping_sub(at) > 300)
                .unwrap_or(true)
        {
            self.desired_state = State::Disarmed;
            self.set_state(now, State::Disarmed, raw);
        }
        self.button(input, raw);
        let due = self
            .last_control
            .map(|at| now.wrapping_sub(at) >= 20)
            .unwrap_or(true);
        if due {
            self.last_control = Some(now);
            if self.role != Role::Remote {
                self.pwm = match self.state {
                    State::Disarmed => {
                        self.previous_pwm = 1035;
                        self.filter.clear();
                        1000
                    }
                    State::Armed => {
                        self.filter.push(throttle::map_mode(raw, self.mode));
                        self.previous_pwm =
                            throttle::ramp(self.filter.average(), self.previous_pwm, self.mode);
                        self.previous_pwm
                    }
                    State::Cruising => {
                        let output = self.cruise_pwm;
                        self.previous_pwm = output;
                        if now.wrapping_sub(self.cruise_at) > 2000
                            && raw >= ((self.cruise_raw as f64) * 0.8) as u16
                        {
                            self.set_state(now, State::Armed, raw);
                        }
                        output
                    }
                };
            }
        }
        let effects = core::mem::take(&mut self.effects);
        Output {
            state: self.state,
            desired_state: self.desired_state,
            pwm: self.pwm,
            send_throttle: due && self.role != Role::Remote && !input.maintenance,
            control_due: due,
            effects,
        }
    }
}
impl Default for Controller {
    fn default() -> Self {
        Self::new(Role::Standalone)
    }
}
