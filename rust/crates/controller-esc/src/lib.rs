//! Production ESC protocol/state machines. Independent of ESP-IDF, shared with host lab.
use controller_core::State;
use controller_dronecan::{DroneCan, Frame, Transfer};
use controller_protocols::fastlink::{FastLink, NO_TEMP};
pub mod firmware;
pub mod relay;
pub use firmware::Flasher;
pub use relay::Relay;
#[derive(Clone, Debug)]
pub struct Request {
    pub id: u16,
    pub payload: Vec<u8>,
}
pub fn signature(id: u16) -> Option<u64> {
    Some(match id {
        5 => 0x569e05394a3017f0,
        200 => 0x8ba32eba566f5713,
        201 => 0x9a5f16e0cdd390f7,
        202 => 0xf8d50afd350a8fbb,
        203 => 0xd1ac3920854f5803,
        209 => 0x65dc98cd51449263,
        213 => 0x81b48c174cda4c7a,
        214 | 218 => 0x40c65964649cae65,
        220 => 0x8764693a96f2952e,
        221 => 0x2eee28c9eb22cf0c,
        222 => 0xa3f15f80ddb60005,
        225 => 0xb970f68e635bad53,
        _ => return None,
    })
}
pub fn restart() -> Request {
    Request {
        id: 5,
        payload: vec![0x1e, 0x1b, 0x55, 0xce, 0xac],
    }
}
pub fn le16(d: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([d[o], d[o + 1]])
}
pub fn i16le(d: &[u8], o: usize) -> i16 {
    i16::from_le_bytes([d[o], d[o + 1]])
}
pub struct Esc {
    pub transport: DroneCan,
    pub telemetry: FastLink,
    pub relay: Relay,
    pub flasher: Flasher,
    tids: [u8; 256],
    last_time: u16,
    last_success: Option<u32>,
    first: bool,
    runtime: u32,
    light: Option<u8>,
}
impl Esc {
    pub fn new() -> Option<Self> {
        let mut transport = DroneCan::new(1)?;
        for id in [
            5, 200, 201, 202, 203, 209, 213, 214, 218, 220, 221, 222, 225,
        ] {
            transport.register(id, 0, 0x20, signature(id)?);
        }
        Some(Self {
            transport,
            telemetry: FastLink::default(),
            relay: Relay::default(),
            flasher: Flasher::default(),
            tids: [0; 256],
            last_time: 0,
            last_success: None,
            first: true,
            runtime: 0,
            light: None,
        })
    }
    pub fn send(&mut self, r: Request) {
        if let Some(signature) = signature(r.id) {
            self.transport.request(
                0x20,
                r.id,
                signature,
                &mut self.tids[r.id as usize],
                &r.payload,
            );
        }
    }
    pub fn throttle(&mut self, pwm: u16) {
        if (1000..=2000).contains(&pwm) && !self.active() {
            self.send(Request {
                id: 213,
                payload: (pwm * 10).to_le_bytes().to_vec(),
            });
        }
    }
    pub fn request_hardware(&mut self) {
        self.send(Request {
            id: 209,
            payload: vec![],
        });
    }
    pub fn receive(&mut self, frame: Frame, now_us: u64) {
        self.transport.receive(frame, now_us);
    }
    pub fn active(&self) -> bool {
        self.relay.active() || self.flasher.active()
    }
    pub fn quiet(&self, now: u32) -> bool {
        self.active() || self.relay.settle(now)
    }
    pub fn dispatch(&mut self, t: Transfer, now: u32) {
        if t.id == 213 && t.data.len() == 33 {
            let d = &t.data;
            let time = le16(d, 28);
            if time != self.last_time {
                if !self.first {
                    self.runtime = self
                        .runtime
                        .wrapping_add(time.wrapping_sub(self.last_time) as u32 * 10);
                }
                self.first = false;
                self.last_time = time;
                self.last_success = Some(now);
                let f = &mut self.telemetry;
                f.esc_runtime_ms = self.runtime;
                f.esc_inPWM = le16(d, 0);
                f.esc_outPWM = le16(d, 2);
                f.esc_rpm = i32::from_le_bytes(d[4..8].try_into().unwrap());
                f.esc_phase_current_dA = i16le(d, 8);
                f.esc_amps_dA = i16le(d, 10);
                f.esc_volts_dV = le16(d, 12);
                f.esc_v_modulation = le16(d, 14);
                f.esc_temp_mos_dC = i16le(d, 16);
                f.esc_temp_cap_dC = i16le(d, 18);
                f.esc_temp_mcu_dC = i16le(d, 20);
                f.esc_error = le16(d, 22);
                f.esc_selfcheck = le16(d, 24);
                let motor = i16le(d, 26);
                f.esc_temp_motor_dC = if (-199..=1400).contains(&motor) {
                    motor
                } else {
                    NO_TEMP
                };
            }
        } else if t.id == 209 && t.data.len() == 26 {
            self.telemetry.esc_hardware_id = le16(&t.data, 0);
            self.telemetry.esc_bootloader_version = le16(&t.data, 2);
            self.telemetry.esc_fw_version = le16(&t.data, 4);
            self.telemetry.esc_sn_code.copy_from_slice(&t.data[10..26]);
        }
        self.relay.response(t.id, &t.data);
        self.flasher.response(t.id, &t.data);
    }
    pub fn tick(&mut self, now: u32, state: State) {
        for _ in 0..32 {
            if let Some(t) = self.transport.take_transfer() {
                self.dispatch(t, now);
            } else {
                break;
            }
        }
        let connected = self
            .last_success
            .map(|at| now.wrapping_sub(at) <= 50)
            .unwrap_or(false);
        if !connected && self.telemetry.esc_status != 0 {
            self.runtime = 0;
            self.first = true;
            self.light = None;
        }
        let was = self.telemetry.esc_status != 0;
        self.telemetry.esc_status = connected as u8;
        if connected && !was && self.telemetry.esc_hardware_id == 0 {
            self.request_hardware();
        }
        self.transport
            .set_node(if self.active() { 0x40 } else { 1 });
        for request in self.relay.tick(now, state) {
            self.send(request);
        }
        for request in self.flasher.tick(now, state) {
            self.send(request);
        }
        self.transport
            .set_node(if self.active() { 0x40 } else { 1 });
        self.transport.cleanup(now as u64 * 1000);
    }
    pub fn motor_tone(&mut self, arm: bool) {
        if self.telemetry.esc_status == 0 || self.active() {
            return;
        }
        self.send(Request {
            id: 214,
            payload: if arm {
                vec![2, 2, 20, 80, 8, 20, 80]
            } else {
                vec![2, 8, 20, 80, 2, 20, 80]
            },
        });
    }
    pub fn status_light(&mut self, state: State, warning: bool, critical: bool) {
        let mode = if self.telemetry.esc_status == 0 {
            0
        } else {
            1 + (state != State::Disarmed) as u8 * 3
                + if critical {
                    2
                } else if warning {
                    1
                } else {
                    0
                }
        };
        if self.light == Some(mode) || self.active() {
            return;
        }
        self.light = Some(mode);
        let mut payload = vec![];
        if mode == 0 {
            payload.extend_from_slice(&[1, 0, 20]);
        } else {
            let color = if critical {
                1
            } else if warning {
                3
            } else {
                2
            };
            if state == State::Disarmed {
                payload.extend_from_slice(&[2, color + 3, 60, 0, 20]);
            } else {
                payload.extend_from_slice(&[4, color, 1, 0, 2, color, 1, 0, 30]);
            }
        }
        self.send(Request { id: 218, payload });
    }
}
