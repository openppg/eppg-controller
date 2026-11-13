//! Deterministic hardware lab using the production Rust controller/protocols.
//! CAN traffic goes through libcanard on both controller and external ESC nodes.
//! This is frame-timed functional simulation, not an RF/electrical simulator.
#[cfg(test)]
mod nor_storage;
use controller_core::monitor::{Data, Monitors};
use controller_core::{Controller, Effects, Input, Role, State};
use controller_dronecan::{DroneCan, Frame};
use controller_esc::{signature, Esc};
use controller_protocols::{ant::AntBms, wireless::Control};
use serde_json::{json, Value};
use std::collections::{BTreeMap, VecDeque};

#[derive(Clone, Debug)]
pub struct Event {
    pub at_ms: u32,
    pub action: String,
    pub value: String,
}
impl Event {
    pub fn new(at_ms: u32, action: &str, value: &str) -> Self {
        Self {
            at_ms,
            action: action.into(),
            value: value.into(),
        }
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct BusStats {
    pub tx_frames: u64,
    pub rx_frames: u64,
    pub tx_bytes: u64,
    pub rx_bytes: u64,
    pub dropped: u64,
    pub corrupted: u64,
    pub overflows: u64,
    pub bus_off: u64,
}
struct Pending {
    frame: Frame,
    destination: u8,
    ready_us: u64,
}
struct Bus {
    bitrate: u32,
    queue: VecDeque<Pending>,
    free_us: u64,
    sequence: u64,
    drop_every: u32,
    corrupt_every: u32,
    off: bool,
    recover_at: Option<u64>,
    stats: BusStats,
}
impl Bus {
    fn new(bitrate: u32) -> Self {
        Self {
            bitrate,
            queue: VecDeque::new(),
            free_us: 0,
            sequence: 0,
            drop_every: 0,
            corrupt_every: 0,
            off: false,
            recover_at: None,
            stats: BusStats::default(),
        }
    }
    fn submit(&mut self, mut frame: Frame, destination: u8, now_us: u64, link: bool) {
        self.sequence += 1;
        self.stats.tx_frames += 1;
        self.stats.tx_bytes += frame.len as u64;
        if !link
            || self.off
            || self.drop_every != 0 && self.sequence.is_multiple_of(self.drop_every as u64)
        {
            self.stats.dropped += 1;
            return;
        }
        if self.queue.len() >= 64 {
            self.stats.overflows += 1;
            self.stats.dropped += 1;
            return;
        }
        if self.corrupt_every != 0
            && self.sequence.is_multiple_of(self.corrupt_every as u64)
            && frame.len > 0
        {
            frame.data[frame.len as usize / 2] ^= 1;
            self.stats.corrupted += 1;
        }
        let bits = 67 + 8 * frame.len as u64 + (34 + 8 * frame.len as u64) / 4;
        self.free_us = self.free_us.max(now_us) + (bits * 1_000_000).div_ceil(self.bitrate as u64);
        self.queue.push_back(Pending {
            frame,
            destination,
            ready_us: self.free_us,
        });
    }
    fn advance(&mut self, now_us: u64) -> Vec<Pending> {
        if self.recover_at.is_some_and(|at| now_us >= at) {
            self.off = false;
            self.recover_at = None;
            self.free_us = now_us;
        }
        let mut out = Vec::new();
        while self.queue.front().is_some_and(|f| f.ready_us <= now_us) {
            let f = self.queue.pop_front().unwrap();
            self.stats.rx_frames += 1;
            self.stats.rx_bytes += f.frame.len as u64;
            out.push(f);
        }
        out
    }
    fn set_off(&mut self, off: bool, now_us: u64) {
        if off {
            if !self.off {
                self.stats.bus_off += 1;
            }
            self.off = true;
            self.recover_at = None;
            self.stats.dropped += self.queue.len() as u64;
            self.queue.clear();
        } else if self.off {
            self.recover_at = Some(now_us + 250_000);
        }
    }
    fn json(&self) -> Value {
        json!({"bitrate":self.bitrate,"tx_frames":self.stats.tx_frames,"rx_frames":self.stats.rx_frames,"tx_bytes":self.stats.tx_bytes,"rx_bytes":self.stats.rx_bytes,"dropped":self.stats.dropped,"corrupted":self.stats.corrupted,"queue_overflows":self.stats.overflows,"bus_off_count":self.stats.bus_off})
    }
}
#[derive(Clone, Debug)]
struct Plant {
    rpm: f32,
    amps: f32,
    voltage: f32,
    temp: f32,
    pwm: u16,
    last_command: u32,
    rpm_override: Option<f32>,
    amps_override: Option<f32>,
    voltage_override: Option<f32>,
}
impl Default for Plant {
    fn default() -> Self {
        Self {
            rpm: 0.,
            amps: 0.,
            voltage: 100.8,
            temp: 25.,
            pwm: 10000,
            last_command: 0,
            rpm_override: None,
            amps_override: None,
            voltage_override: None,
        }
    }
}
impl Plant {
    fn advance(&mut self, now: u32) {
        let power = if now.wrapping_sub(self.last_command) <= 300 {
            ((self.pwm as f32 - 10000.) / 9500.).clamp(0., 1.)
        } else {
            0.
        };
        self.rpm = self
            .rpm_override
            .unwrap_or(self.rpm + (power * 12000. - self.rpm) * 0.12);
        self.amps = self
            .amps_override
            .unwrap_or(self.amps + (power * power * 120. - self.amps) * 0.15);
        self.voltage = self.voltage_override.unwrap_or(100.8 - self.amps * 0.012);
    }
    fn response(&self, now: u32) -> [u8; 33] {
        let mut b = [0; 33];
        b[0..2].copy_from_slice(&self.pwm.to_le_bytes());
        b[2..4].copy_from_slice(&self.pwm.to_le_bytes());
        b[4..8].copy_from_slice(&(self.rpm.round() as i32).to_le_bytes());
        for (o, v) in [
            (8, (self.amps * 17.).round() as i16),
            (10, (self.amps * 10.).round() as i16),
            (16, (self.temp * 10.).round() as i16),
            (18, ((self.temp - 3.) * 10.).round() as i16),
            (20, ((self.temp - 7.) * 10.).round() as i16),
            (26, ((self.temp + 5.) * 10.).round() as i16),
        ] {
            b[o..o + 2].copy_from_slice(&v.to_le_bytes());
        }
        b[12..14].copy_from_slice(&((self.voltage * 10.).round() as u16).to_le_bytes());
        b[14..16].copy_from_slice(&100u16.to_le_bytes());
        b[28..30].copy_from_slice(&((now / 10) as u16).to_le_bytes());
        b
    }
}

pub struct Simulator {
    pub controller: Controller,
    pub esc: Esc,
    pub bms: AntBms,
    pub monitors: Monitors,
    pub now_ms: u32,
    peer: DroneCan,
    esc_bus: Bus,
    bms_bus: Bus,
    bms_rx: VecDeque<Frame>,
    plant: Plant,
    peer_parameters: BTreeMap<u16, Vec<u8>>,
    peer_saved_parameters: BTreeMap<u16, Vec<u8>>,
    peer_flash: Vec<u8>,
    peer_flash_started: bool,
    peer_flash_complete: bool,
    peer_next_chunk: u16,
    peer_rejected_sources: u64,
    events: Vec<Event>,
    next_event: usize,
    has_run: bool,
    pub raw: u16,
    button: bool,
    maintenance: bool,
    esc_link: bool,
    bms_link: bool,
    baro_link: bool,
    stalled: bool,
    last_control: u32,
    requested_soc: f32,
    bms_temp: f32,
    bms_voltage: Option<f32>,
    bms_current: Option<f32>,
    altitude: f32,
    vario: f32,
    previous_altitude: f32,
    cpu_temp: f32,
    altitude_history: VecDeque<(u32, f32)>,
    pub watchdog_resets: u32,
    pub ble_notifications: u64,
    pub effects: u64,
    pub logs: Vec<String>,
}
impl Simulator {
    pub fn new() -> Result<Self, String> {
        let esc = Esc::new().ok_or("cannot allocate controller CAN node")?;
        let mut peer = DroneCan::new(0x20).ok_or("cannot allocate ESC CAN node")?;
        for id in [
            5, 200, 201, 202, 203, 209, 213, 214, 218, 220, 221, 222, 225,
        ] {
            peer.register(id, 1, 0, signature(id).unwrap());
        }
        Ok(Self {
            controller: Controller::new(Role::Standalone),
            esc,
            bms: AntBms::default(),
            monitors: Monitors::default(),
            now_ms: 0,
            peer,
            esc_bus: Bus::new(1_000_000),
            bms_bus: Bus::new(250_000),
            bms_rx: VecDeque::new(),
            plant: Plant::default(),
            peer_parameters: BTreeMap::new(),
            peer_saved_parameters: BTreeMap::new(),
            peer_flash: Vec::new(),
            peer_flash_started: false,
            peer_flash_complete: false,
            peer_next_chunk: 0,
            peer_rejected_sources: 0,
            events: Vec::new(),
            next_event: 0,
            has_run: false,
            raw: 0,
            button: false,
            maintenance: false,
            esc_link: true,
            bms_link: true,
            baro_link: true,
            stalled: false,
            last_control: 0,
            requested_soc: 90.,
            bms_temp: 25.,
            bms_voltage: None,
            bms_current: None,
            altitude: 0.,
            vario: 0.,
            previous_altitude: 0.,
            cpu_temp: 42.,
            altitude_history: VecDeque::new(),
            watchdog_resets: 0,
            ble_notifications: 0,
            effects: 0,
            logs: Vec::new(),
        })
    }
    pub fn schedule(&mut self, event: Event) -> Result<(), String> {
        if self.has_run && event.at_ms <= self.now_ms {
            return Err("event time must be after current simulator time".into());
        }
        self.events.push(event);
        self.events.sort_by_key(|e| e.at_ms);
        Ok(())
    }
    fn log(&mut self, text: impl Into<String>) {
        self.logs.push(format!("{}ms {}", self.now_ms, text.into()));
    }
    fn number(value: &str) -> Result<f32, String> {
        let n = value
            .parse::<f32>()
            .map_err(|_| format!("invalid number '{value}'"))?;
        if !n.is_finite() {
            return Err(format!("non-finite number '{value}'"));
        }
        Ok(n)
    }
    fn boolean(value: &str) -> Result<bool, String> {
        match value {
            "1" | "true" | "on" | "down" => Ok(true),
            "0" | "false" | "off" | "up" => Ok(false),
            _ => Err(format!("invalid boolean '{value}'")),
        }
    }
    fn action(&mut self, event: Event) -> Result<(), String> {
        let v = event.value.as_str();
        match event.action.as_str() {
            "throttle" => self.raw = Self::number(v)?.clamp(0., 4095.) as u16,
            "button" => self.button = Self::boolean(v)?,
            "arm" => {
                let armed = Self::boolean(v)?;
                if (self.controller.state != State::Disarmed) != armed {
                    self.controller.toggle_arm(
                        self.now_ms,
                        self.raw,
                        self.maintenance || self.esc.active(),
                    );
                }
            }
            "cruise" => {
                let cruising = Self::boolean(v)?;
                if (self.controller.state == State::Cruising) != cruising {
                    self.controller.toggle_cruise(
                        self.now_ms,
                        self.raw,
                        self.maintenance || self.esc.active(),
                    );
                }
            }
            "mode" => {
                self.controller.mode = match v {
                    "chill" | "0" => 0,
                    "sport" | "1" => 1,
                    _ => return Err("mode must be chill or sport".into()),
                }
            }
            "role" => {
                let role = match v {
                    "standalone" => Role::Standalone,
                    "remote" => Role::Remote,
                    "receiver" => Role::Receiver,
                    _ => return Err("unknown role".into()),
                };
                self.controller.set_role(role);
            }
            "maintenance" => self.maintenance = Self::boolean(v)?,
            "esc_link" => self.esc_link = Self::boolean(v)?,
            "bms_link" => self.bms_link = Self::boolean(v)?,
            "barometer_link" => self.baro_link = Self::boolean(v)?,
            "esc_rpm" => self.plant.rpm_override = Some(Self::number(v)?),
            "esc_amps" => self.plant.amps_override = Some(Self::number(v)?),
            "esc_voltage" => self.plant.voltage_override = Some(Self::number(v)?),
            "esc_temp" => self.plant.temp = Self::number(v)?,
            "soc" => self.requested_soc = Self::number(v)?.clamp(0., 100.),
            "bms_temp" => self.bms_temp = Self::number(v)?,
            "bms_voltage" => self.bms_voltage = Some(Self::number(v)?),
            "bms_current" => self.bms_current = Some(Self::number(v)?),
            "cpu_temp" => self.cpu_temp = Self::number(v)?,
            "altitude" => self.altitude = Self::number(v)?,
            "stall_throttle" => self.stalled = Self::boolean(v)?,
            "esc_drop_every" => {
                self.esc_bus.drop_every = v.parse().map_err(|_| "invalid drop rate")?
            }
            "bms_drop_every" => {
                self.bms_bus.drop_every = v.parse().map_err(|_| "invalid drop rate")?
            }
            "esc_corrupt_every" => {
                self.esc_bus.corrupt_every = v.parse().map_err(|_| "invalid corruption rate")?
            }
            "bms_corrupt_every" => {
                self.bms_bus.corrupt_every = v.parse().map_err(|_| "invalid corruption rate")?
            }
            "esc_bus_off" => self
                .esc_bus
                .set_off(Self::boolean(v)?, self.now_ms as u64 * 1000),
            "bms_bus_off" => self
                .bms_bus
                .set_off(Self::boolean(v)?, self.now_ms as u64 * 1000),
            "wireless_control" => {
                let p: Vec<_> = v.split(':').collect();
                if p.len() != 5 {
                    return Err("wireless_control requires session:sequence:raw:state:mode".into());
                }
                let c = Control {
                    session: p[0].parse().map_err(|_| "invalid session")?,
                    sequence: p[1].parse().map_err(|_| "invalid sequence")?,
                    throttle: p[2].parse().map_err(|_| "invalid throttle")?,
                    state: p[3].parse().map_err(|_| "invalid state")?,
                    mode: p[4].parse().map_err(|_| "invalid mode")?,
                    uptime: self.now_ms,
                    ..Control::default()
                };
                if let Some(c) = Control::decode(&c.encode()) {
                    self.controller.receive_control(
                        self.now_ms,
                        c.session,
                        c.sequence,
                        c.throttle,
                        State::from_byte(c.state).unwrap(),
                        c.mode,
                    );
                }
            }
            _ => return Err(format!("unsupported action '{}'", event.action)),
        }
        Ok(())
    }
    fn drain_controller(&mut self) {
        while let Some(frame) = self.esc.transport.peek_frame() {
            self.esc_bus
                .submit(frame, 1, self.now_ms as u64 * 1000, self.esc_link);
            self.esc.transport.pop_frame();
        }
    }
    fn drain_peer(&mut self) {
        while let Some(frame) = self.peer.peek_frame() {
            self.esc_bus
                .submit(frame, 0, self.now_ms as u64 * 1000, self.esc_link);
            self.peer.pop_frame();
        }
    }
    fn wire(&mut self) {
        let now = self.now_ms as u64 * 1000;
        for pending in self.esc_bus.advance(now) {
            if pending.destination == 0 {
                self.esc.receive(pending.frame, pending.ready_us);
            } else {
                self.peer.receive(pending.frame, pending.ready_us);
            }
        }
        while let Some(t) = self.peer.take_transfer() {
            // Real ESC config/bootloader services require host node 0x40.
            if matches!(t.id,200..=203|220..=225) && t.source != 0x40 {
                self.peer_rejected_sources += 1;
                continue;
            }
            let reply = match t.id {
                213 if t.data.len() == 2 => {
                    self.plant.pwm = u16::from_le_bytes([t.data[0], t.data[1]]);
                    self.plant.last_command = self.now_ms;
                    Some(self.plant.response(self.now_ms).to_vec())
                }
                209 => {
                    let mut payload = vec![0; 26];
                    payload[..2].copy_from_slice(&1u16.to_le_bytes());
                    payload[2..4].copy_from_slice(&1u16.to_le_bytes());
                    payload[4..6].copy_from_slice(&801u16.to_le_bytes());
                    payload[10..].copy_from_slice(b"SIM-ESC-00000001");
                    Some(payload)
                }
                5 => {
                    self.peer_parameters = self.peer_saved_parameters.clone();
                    Some(vec![1])
                }
                225 => Some(vec![0]),
                220 if t.data.len() >= 2 => {
                    let id = u16::from_le_bytes([t.data[0], t.data[1]]);
                    self.peer_parameters.insert(id, t.data[2..].to_vec());
                    let mut payload = id.to_le_bytes().to_vec();
                    payload.push(0);
                    Some(payload)
                }
                221 if t.data.len() >= 2 => {
                    let id = u16::from_le_bytes([t.data[0], t.data[1]]);
                    let mut payload = id.to_le_bytes().to_vec();
                    payload.push(0);
                    payload.extend_from_slice(
                        self.peer_parameters
                            .get(&id)
                            .map(Vec::as_slice)
                            .unwrap_or(&[0, 0]),
                    );
                    Some(payload)
                }
                222 => {
                    self.peer_saved_parameters = self.peer_parameters.clone();
                    Some(vec![0])
                }
                200 => Some(vec![2]),
                201 => {
                    self.peer_flash.clear();
                    self.peer_flash_started = true;
                    self.peer_flash_complete = false;
                    self.peer_next_chunk = 0;
                    Some(vec![0])
                }
                202 if t.data.len() == 258 && self.peer_flash_started => {
                    let index = u16::from_le_bytes([t.data[0], t.data[1]]);
                    if index == self.peer_next_chunk {
                        self.peer_flash.extend_from_slice(&t.data[2..]);
                        self.peer_next_chunk += 1;
                    }
                    let mut payload = vec![0];
                    payload.extend_from_slice(&self.peer_next_chunk.to_le_bytes());
                    Some(payload)
                }
                203 => {
                    self.peer_flash_complete = self.peer_flash_started;
                    self.peer_flash_started = false;
                    Some(vec![0])
                }
                214 | 218 => Some(vec![0]),
                _ => None,
            };
            if let Some(payload) = reply {
                self.peer.respond(
                    t.source,
                    t.id,
                    signature(t.id).unwrap(),
                    t.transfer_id,
                    &payload,
                );
            }
        }
        self.drain_peer();
        for pending in self.bms_bus.advance(now) {
            if self.bms_rx.len() < 64 {
                self.bms_rx.push_back(pending.frame);
            } else {
                self.bms_bus.stats.overflows += 1;
            }
        }
    }
    fn publish_bms(&mut self) {
        let current = self.bms_current.unwrap_or(self.plant.amps);
        let cell = 3. + 1.2 * self.requested_soc / 100.;
        let voltage = self.bms_voltage.unwrap_or(24. * cell - current * 0.012);
        let amps = (5000. + current * 10.).round().clamp(0., 65535.) as u16;
        let volts = (voltage * 10.).round().clamp(0., 65535.) as u16;
        let max = ((cell + 0.012) * 1000.).round() as u16;
        let min = ((cell - 0.010) * 1000.).round() as u16;
        let temp = (self.bms_temp + 40.).round().clamp(0., 255.) as u8;
        let mut frames = vec![
            (
                0x18ff28f4,
                [
                    0x38,
                    self.requested_soc.round() as u8,
                    amps as u8,
                    (amps >> 8) as u8,
                    volts as u8,
                    (volts >> 8) as u8,
                    0,
                    0,
                ],
            ),
            (
                0x18fe28f4,
                [
                    max as u8,
                    (max >> 8) as u8,
                    min as u8,
                    (min >> 8) as u8,
                    temp,
                    temp,
                    0,
                    0,
                ],
            ),
            (0x18b428f4, [temp, temp, temp, temp, temp, temp, 0, 0]),
        ];
        for group in 0..6 {
            let mv = (cell * 1000.).round() as u16;
            let mut data = [0; 8];
            for i in 0..4 {
                data[i * 2..i * 2 + 2].copy_from_slice(&mv.to_le_bytes());
            }
            frames.push((0x18c828f4 + (group << 16), data));
        }
        for (id, data) in frames {
            self.bms_bus.submit(
                Frame { id, data, len: 8 },
                2,
                self.now_ms as u64 * 1000,
                self.bms_link,
            );
        }
    }
    fn monitor(&mut self) {
        let e = &self.esc.telemetry;
        let b = self.bms.snapshot(self.now_ms);
        self.monitors.update(
            self.now_ms,
            Data {
                esc_connected: e.esc_status != 0,
                bms_connected: b.bms_status != 0,
                armed: self.controller.state != State::Disarmed,
                esc_initialized: true,
                bms_initialized: true,
                baro_initialized: self.baro_link,
                esc_temps: [
                    e.esc_temp_mos_dC as f32 / 10.,
                    e.esc_temp_mcu_dC as f32 / 10.,
                    e.esc_temp_cap_dC as f32 / 10.,
                    if e.esc_temp_motor_dC == i16::MIN {
                        f32::NAN
                    } else {
                        e.esc_temp_motor_dC as f32 / 10.
                    },
                ],
                running_errors: e.esc_error,
                selfcheck_errors: e.esc_selfcheck,
                bms_temps: self.bms.temperatures(),
                pack_voltage: b.bms_volts_dV as f32 / 10.,
                max_cell: b.bms_cell_max_mV as f32 / 1000.,
                min_cell: b.bms_cell_min_mV as f32 / 1000.,
                soc: b.bms_soc as f32,
                voltage_diff: b.bms_voltage_diff_mV as f32 / 1000.,
                charge_mos: b.bms_is_charge_mos != 0,
                discharge_mos: b.bms_is_discharge_mos != 0,
                baro_temp: 25.,
                cpu_temp: self.cpu_temp,
            },
        );
    }
    pub fn run(
        &mut self,
        duration: u32,
        mut trace: Option<&mut dyn std::io::Write>,
    ) -> Result<(), String> {
        if duration < self.now_ms {
            return Err("duration precedes current virtual time".into());
        }
        let start = if self.has_run {
            self.now_ms.saturating_add(1)
        } else {
            0
        };
        for now in start..=duration {
            self.now_ms = now;
            while self
                .events
                .get(self.next_event)
                .is_some_and(|e| e.at_ms == now)
            {
                let event = self.events[self.next_event].clone();
                self.next_event += 1;
                self.action(event)?;
            }
            self.wire();
            if now % 20 == 0 {
                self.plant.advance(now);
            }
            if now % 10 == 0 && !self.stalled {
                let before = self.controller.state;
                let output = self.controller.tick(Input {
                    now_ms: now,
                    raw: self.raw,
                    button_down: self.button,
                    maintenance: self.maintenance || self.esc.active(),
                });
                self.effects += output.effects.0.count_ones() as u64;
                if output.effects.contains(Effects::ZERO_ALTITUDE) {
                    self.previous_altitude = self.altitude;
                }
                if self.controller.state != before {
                    self.log(format!("state -> {:?}", self.controller.state));
                }
                if output.control_due {
                    self.last_control = now;
                    if output.send_throttle {
                        self.esc.throttle(output.pwm);
                    }
                    self.esc.tick(now, self.controller.state);
                    self.drain_controller();
                }
            }
            if now % 100 == 0 {
                while let Some(f) = self.bms_rx.pop_front() {
                    self.bms.packet(f.id, &f.data[..f.len as usize], now);
                }
                if self.controller.role != Role::Remote {
                    self.publish_bms();
                }
                self.monitor();
                if now.wrapping_sub(self.last_control) >= 5000 {
                    self.watchdog_resets += 1;
                    let mode = self.controller.mode;
                    let role = self.controller.role;
                    self.controller = Controller::new(role);
                    self.controller.mode = mode;
                    self.stalled = false;
                    self.last_control = now;
                    self.log("watchdog reset: throttle task stalled");
                }
            }
            if now % 20 == 0 {
                self.ble_notifications += 1;
            }
            if now % 33 == 0 {
                self.altitude_history.push_back((now, self.altitude));
                while self.altitude_history.len() > 50 {
                    self.altitude_history.pop_front();
                }
                if let Some((at, alt)) = self.altitude_history.front() {
                    if now > *at {
                        self.vario = ((self.altitude - *alt) / ((now - *at) as f32 / 1000.))
                            .clamp(-250., 250.);
                    }
                }
                if let Some(writer) = trace.as_deref_mut() {
                    writeln!(writer, "{}", self.snapshot()).map_err(|e| e.to_string())?;
                }
            }
        }
        self.has_run = true;
        Ok(())
    }
    pub fn telemetry(&self) -> controller_protocols::fastlink::FastLink {
        let mut bytes = self.bms.snapshot(self.now_ms).encode();
        bytes[..79].copy_from_slice(&self.esc.telemetry.encode()[..79]);
        let mut packet = controller_protocols::fastlink::FastLink::decode(&bytes)
            .expect("known fixed-width codec");
        packet.uptime_ms = self.now_ms;
        packet.packet_id = self.ble_notifications as u32;
        packet.pot_raw = self.raw;
        packet.device_state = self.controller.state as u8;
        packet.altitude_cm = (self.altitude * 100.).round() as i32;
        packet.vario_cmps = (self.vario * 100.).round() as i16;
        packet.baro_temp_dC = 250;
        packet.baro_pressure_dHPa =
            (10132.5 * (1. - self.altitude / 44330.).powf(5.255)).round() as u16;
        packet.mcu_temp_dC = (self.cpu_temp * 10.).round() as i16;
        let masks = self.monitors.masks();
        packet.esc_warning_alerts = masks[0];
        packet.esc_critical_alerts = masks[1];
        packet.bms_warning_alerts = masks[2];
        packet.bms_critical_alerts = masks[3];
        packet
    }
    /// Render the same Rust-owned LVGL screen shipped in the firmware.
    pub fn render_bmp(&self, path: &std::path::Path, dark: bool) -> Result<(), String> {
        let settings = controller_core::settings::Settings {
            theme: dark as u8,
            performance_mode: self.controller.mode,
            ..Default::default()
        };
        let mut ui = controller_ui::Ui::with_settings(&settings);
        ui.set_armed_start(self.controller.armed_at);
        ui.set_connected(true);
        ui.set_monitors(&self.monitors);
        ui.update(&self.telemetry(), &settings, false, self.now_ms);
        ui.render_at(self.now_ms);
        let pixels = ui.framebuffer();
        if pixels.len() != 160 * 128 {
            return Err("native framebuffer size mismatch".into());
        }
        let row = 160 * 3;
        let size = 54 + row * 128;
        let mut output = vec![0u8; size];
        output[..2].copy_from_slice(b"BM");
        output[2..6].copy_from_slice(&(size as u32).to_le_bytes());
        output[10..14].copy_from_slice(&54u32.to_le_bytes());
        output[14..18].copy_from_slice(&40u32.to_le_bytes());
        output[18..22].copy_from_slice(&160u32.to_le_bytes());
        output[22..26].copy_from_slice(&128u32.to_le_bytes());
        output[26..28].copy_from_slice(&1u16.to_le_bytes());
        output[28..30].copy_from_slice(&24u16.to_le_bytes());
        output[34..38].copy_from_slice(&((row * 128) as u32).to_le_bytes());
        for y in 0..128 {
            for x in 0..160 {
                let color = pixels[y * 160 + x];
                let offset = 54 + (127 - y) * row + x * 3;
                output[offset] = ((color & 31) << 3) as u8;
                output[offset + 1] = (((color >> 5) & 63) << 2) as u8;
                output[offset + 2] = (((color >> 11) & 31) << 3) as u8;
            }
        }
        std::fs::write(path, output).map_err(|e| e.to_string())
    }

    pub fn snapshot(&self) -> Value {
        let e = &self.esc.telemetry;
        let b = self.bms.snapshot(self.now_ms);
        let s = self.esc_bus.stats;
        let t = self.bms_bus.stats;
        let labels = ["ESC_MOS_TEMP", "ESC_MCU_TEMP", "ESC_CAP_TEMP", "MOTOR_TEMP"];
        let alerts: Vec<String> = self
            .monitors
            .levels
            .iter()
            .enumerate()
            .filter(|(_, l)| l.critical() || l.warning())
            .map(|(id, l)| {
                let name = match id {
                    0..=3 => labels[id],
                    25 => "ESC_CAN_LINK_LOST",
                    26..=31 => "BMS_TEMP",
                    34 => "BMS_SOC",
                    40 => "BMS_CAN_LINK_LOST",
                    42 => "BARO_INIT_FAILURE",
                    43 => "CPU_TEMP",
                    _ => "SENSOR",
                };
                if id == 25 || id == 40 || id == 42 {
                    name.into()
                } else {
                    format!("{name}_{}", if l.critical() { "CRIT" } else { "WARN" })
                }
            })
            .collect();
        json!({"backend":"rust-production-core","time_ms":self.now_ms,"state":match self.controller.state{State::Disarmed=>"DISARMED",State::Armed=>"ARMED",State::Cruising=>"ARMED_CRUISING"},"mode":self.controller.mode,"role":self.controller.role as u8,"throttle_raw":self.raw,"throttle_pwm":self.controller.pwm,
   "esc":{"connected":e.esc_status!=0,"voltage_v":e.esc_volts_dV as f32/10.,"rpm":e.esc_rpm,"amps":e.esc_amps_dA as f32/10.,"mos_temp_c":e.esc_temp_mos_dC as f32/10.},
   "bms":{"connected":b.bms_status!=0,"voltage_v":b.bms_volts_dV as f32/10.,"current_a":b.bms_amps_dA as f32/10.,"soc_pct":b.bms_soc,"temp_c":b.bms_highest_temp_C},
   "barometer":{"connected":self.baro_link,"altitude_m":self.altitude,"vertical_speed_mps":self.vario,"pressure_hpa":1013.25*(1.-self.altitude/44330.).powf(5.255)},
   "plants":{"esc":{"rpm":self.plant.rpm,"amps":self.plant.amps,"pwm_01us":self.plant.pwm,"command_age_ms":self.now_ms.wrapping_sub(self.plant.last_command)}},"can":{"esc":self.esc_bus.json(),"bms":self.bms_bus.json()},"io":{"can_tx_frames":s.tx_frames+t.tx_frames,"can_rx_frames":s.rx_frames+t.rx_frames,"can_tx_bytes":s.tx_bytes+t.tx_bytes,"can_rx_bytes":s.rx_bytes+t.rx_bytes,"can_dropped_frames":s.dropped+t.dropped,"can_corrupted_frames":s.corrupted+t.corrupted,"can_bus_off_count":s.bus_off+t.bus_off,"ble_notifications":self.ble_notifications},
   "maintenance":{"config_code":self.esc.relay.status.code,"config_phase":self.esc.relay.status.phase as u8,"firmware_code":self.esc.flasher.code,"firmware_phase":self.esc.flasher.phase as u8,"rejected_can_sources":self.peer_rejected_sources,"flashed_bytes":self.peer_flash.len(),"flash_complete":self.peer_flash_complete},"watchdog_resets":self.watchdog_resets,"effects":self.effects,"alerts":alerts,"alert_masks":self.monitors.masks(),"fidelity":{"control":"production Rust","CAN":"production libcanard and ANT parser, frame-timed buses","RF":"application protocol only","clock":"virtual milliseconds"}})
    }
}

pub fn parse_scenario(text: &str) -> Result<Vec<Event>, String> {
    let mut out = Vec::new();
    let mut last = 0;
    for (line_no, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let f: Vec<_> = line.split(',').map(str::trim).collect();
        if f.len() != 3 {
            return Err(format!(
                "line{}: expected time_ms,action,value",
                line_no + 1
            ));
        }
        let at = f[0]
            .parse::<u32>()
            .map_err(|_| format!("line{}: invalid time", line_no + 1))?;
        if at < last {
            return Err(format!("line{}: events must be ordered", line_no + 1));
        }
        last = at;
        out.push(Event::new(at, f[1], f[2]));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn event(sim: &mut Simulator, t: u32, a: &str, v: &str) {
        sim.schedule(Event::new(t, a, v)).unwrap();
    }
    fn flight() -> Simulator {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 0, "mode", "sport");
        event(&mut s, 600, "arm", "on");
        event(&mut s, 800, "throttle", "3000");
        s
    }
    #[test]
    fn production_controller_ramps_across_real_can() {
        let mut s = flight();
        s.run(2000, None).unwrap();
        assert_eq!(s.controller.state, State::Armed);
        assert!(s.controller.pwm > 1400);
        assert_eq!(s.esc.telemetry.esc_status, 1);
        assert!(s.esc.telemetry.esc_rpm > 1000);
        assert!(s.esc_bus.stats.rx_frames > 0);
        assert!(s.bms.connected(2000));
    }
    #[test]
    fn unsafe_arm_and_boot_cooldown_match_firmware() {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 100, "arm", "on");
        s.run(200, None).unwrap();
        assert_eq!(s.controller.state, State::Disarmed);
        event(&mut s, 600, "throttle", "204");
        event(&mut s, 600, "arm", "on");
        s.run(700, None).unwrap();
        assert_eq!(s.controller.state, State::Disarmed);
        event(&mut s, 800, "throttle", "203");
        event(&mut s, 800, "arm", "on");
        s.run(900, None).unwrap();
        assert_eq!(s.controller.state, State::Armed);
    }
    #[test]
    fn wire_does_not_deliver_instantly() {
        let mut s = Simulator::new().unwrap();
        s.run(0, None).unwrap();
        assert_eq!(s.esc.telemetry.esc_status, 0);
        assert!(!s.bms.connected(0));
        s.run(100, None).unwrap();
        assert_eq!(s.esc.telemetry.esc_status, 1);
        assert!(s.bms.connected(100));
    }
    #[test]
    fn corrupted_dronecan_is_rejected_by_production_transport() {
        let mut s = flight();
        event(&mut s, 1500, "esc_corrupt_every", "2");
        s.run(2200, None).unwrap();
        assert!(s.esc_bus.stats.corrupted > 0);
        assert_eq!(s.esc.telemetry.esc_status, 0);
        assert!(s.snapshot()["alerts"]
            .as_array()
            .unwrap()
            .iter()
            .any(|a| a == "ESC_CAN_LINK_LOST"));
    }
    #[test]
    fn frame_drop_disconnects_and_debounces() {
        let mut s = flight();
        event(&mut s, 1500, "esc_drop_every", "1");
        s.run(1900, None).unwrap();
        assert_eq!(s.monitors.levels[25], controller_core::monitor::Level::Ok);
        s.run(2200, None).unwrap();
        assert!(s.monitors.levels[25].critical());
        assert!(s.esc_bus.stats.dropped > 0);
    }
    #[test]
    fn recovered_bus_waits_250ms_before_new_frames() {
        let mut s = flight();
        event(&mut s, 1500, "esc_bus_off", "on");
        event(&mut s, 1800, "esc_bus_off", "off");
        s.run(2000, None).unwrap();
        assert_eq!(s.esc.telemetry.esc_status, 0);
        s.run(2200, None).unwrap();
        assert_eq!(s.esc.telemetry.esc_status, 1);
        assert_eq!(s.esc_bus.stats.bus_off, 1);
    }
    #[test]
    fn esc_timeout_precedes_watchdog_recovery() {
        let mut s = flight();
        event(&mut s, 1500, "stall_throttle", "on");
        s.run(4500, None).unwrap();
        assert_eq!(s.watchdog_resets, 0);
        assert!(
            s.plant.rpm < 1.,
            "ESC plant continued spinning after its own command timeout"
        );
        s.run(6600, None).unwrap();
        assert_eq!(s.watchdog_resets, 1);
        assert_eq!(s.controller.state, State::Disarmed);
        assert_eq!(s.controller.pwm, 1000);
    }
    #[test]
    fn maintenance_prevents_arming() {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 0, "maintenance", "on");
        event(&mut s, 600, "arm", "on");
        s.run(1000, None).unwrap();
        assert_eq!(s.controller.state, State::Disarmed);
    }
    #[test]
    fn click_hold_uses_production_button_state_machine() {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 100, "button", "down");
        event(&mut s, 200, "button", "up");
        event(&mut s, 400, "button", "down");
        event(&mut s, 2600, "button", "up");
        s.run(2390, None).unwrap();
        assert_eq!(s.controller.state, State::Disarmed);
        s.run(2410, None).unwrap();
        assert_eq!(s.controller.state, State::Armed);
    }
    #[test]
    fn cruise_override_obeys_two_second_grace() {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 600, "arm", "on");
        event(&mut s, 800, "throttle", "1000");
        event(&mut s, 1000, "cruise", "on");
        s.run(3000, None).unwrap();
        assert_eq!(s.controller.state, State::Cruising);
        s.run(3020, None).unwrap();
        assert_eq!(s.controller.state, State::Armed);
    }
    #[test]
    fn receiver_rejects_unarmed_session_and_fails_safe() {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 0, "role", "receiver");
        event(&mut s, 600, "wireless_control", "12:1:0:1:1");
        s.run(610, None).unwrap();
        assert_eq!(s.controller.state, State::Disarmed);
        event(&mut s, 620, "wireless_control", "12:2:0:0:1");
        event(&mut s, 640, "wireless_control", "12:3:0:1:1");
        s.run(650, None).unwrap();
        assert_eq!(s.controller.state, State::Armed);
        s.run(810, None).unwrap();
        assert_eq!(s.controller.state, State::Disarmed);
        assert!(s.controller.failsafe);
    }
    #[test]
    fn receiver_rejects_old_sequence_without_refreshing_timeout() {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 0, "role", "receiver");
        event(&mut s, 600, "wireless_control", "1:10:0:0:0");
        event(&mut s, 620, "wireless_control", "1:11:0:1:0");
        event(&mut s, 700, "wireless_control", "1:10:0:1:0");
        s.run(790, None).unwrap();
        assert_eq!(s.controller.state, State::Disarmed);
        assert!(s.controller.failsafe);
    }
    #[test]
    fn bms_startup_grace_and_real_sensor_thresholds() {
        let mut s = Simulator::new().unwrap();
        event(&mut s, 0, "soc", "3");
        event(&mut s, 0, "esc_temp", "120");
        s.run(2000, None).unwrap();
        assert!(s.monitors.levels[0].critical());
        assert_eq!(s.monitors.levels[34], controller_core::monitor::Level::Ok);
        s.run(2200, None).unwrap();
        assert!(s.monitors.levels[34].critical());
    }
    #[test]
    fn injected_telemetry_traverses_wire_codecs() {
        let mut s = Simulator::new().unwrap();
        for (a, v) in [
            ("esc_rpm", "4321"),
            ("esc_amps", "37.4"),
            ("esc_voltage", "91.2"),
            ("bms_voltage", "89.6"),
            ("bms_current", "34.2"),
            ("soc", "64"),
            ("bms_temp", "38"),
        ] {
            event(&mut s, 0, a, v);
        }
        s.run(300, None).unwrap();
        assert_eq!(s.esc.telemetry.esc_rpm, 4321);
        assert_eq!(s.esc.telemetry.esc_amps_dA, 374);
        assert_eq!(s.esc.telemetry.esc_volts_dV, 912);
        let b = s.bms.snapshot(300);
        assert_eq!(b.bms_volts_dV, 896);
        assert_eq!(b.bms_amps_dA, 342);
        assert_eq!(b.bms_soc, 64);
        assert_eq!(b.bms_highest_temp_C, 38);
    }
    #[test]
    fn non_finite_and_out_of_order_inputs_rejected() {
        assert!(parse_scenario("10,arm,on\n9,arm,off").is_err());
        let mut s = Simulator::new().unwrap();
        event(&mut s, 0, "altitude", "nan");
        assert!(s.run(1, None).is_err());
    }
    #[test]
    fn virtual_time_trace_is_repeatable() {
        let mut a = flight();
        let mut b = flight();
        let mut trace_a = Vec::new();
        let mut trace_b = Vec::new();
        a.run(2000, Some(&mut trace_a)).unwrap();
        b.run(2000, Some(&mut trace_b)).unwrap();
        assert_eq!(trace_a, trace_b);
        assert_eq!(a.snapshot(), b.snapshot());
        assert_eq!(trace_a.iter().filter(|b| **b == b'\n').count(), 61);
    }
}

#[cfg(test)]
mod maintenance_tests {
    use super::*;
    #[test]
    fn config_write_save_restart_and_verify_cross_real_can() {
        let mut sim = Simulator::new().unwrap();
        sim.run(500, None).unwrap();
        assert!(sim
            .esc
            .relay
            .command(0x10, &[0x10, 0x01, 2, 0x34, 0x12], false));
        sim.schedule(Event::new(600, "arm", "on")).unwrap();
        sim.run(7000, None).unwrap();
        assert_eq!(sim.controller.state, State::Disarmed);
        assert_eq!(sim.esc.relay.status.code, 4, "{:?}", sim.esc.relay.status);
        assert_eq!(
            sim.peer_saved_parameters.get(&0x0110).unwrap(),
            &[0x34, 0x12]
        );
        assert_eq!(
            sim.peer_rejected_sources, 0,
            "session requests used the wrong CAN host node"
        );
    }
    #[test]
    fn read_all_uses_production_parameter_manifest_and_result_codec() {
        let mut sim = Simulator::new().unwrap();
        sim.run(500, None).unwrap();
        assert!(sim.esc.relay.command(0x30, &[], false));
        sim.run(7000, None).unwrap();
        assert_eq!(sim.esc.relay.status.code, 7, "{:?}", sim.esc.relay.status);
        assert!(sim.esc.relay.result.len() > 600);
        assert_eq!(sim.esc.relay.status.id as usize, sim.esc.relay.result.len());
        assert_eq!(sim.peer_rejected_sources, 0);
    }
    #[test]
    fn config_batch_persists_and_verifies_each_parameter() {
        let mut sim = Simulator::new().unwrap();
        sim.run(500, None).unwrap();
        assert!(sim.esc.relay.command(0x40, &[], false));
        assert!(sim
            .esc
            .relay
            .command(0x41, &[0x10, 0x01, 2, 0x34, 0x12], false));
        assert!(sim
            .esc
            .relay
            .command(0x41, &[0x11, 0x01, 2, 0x78, 0x56], false));
        assert!(sim.esc.relay.command(0x42, &[], false));
        sim.run(7500, None).unwrap();
        assert_eq!(sim.esc.relay.status.code, 4, "{:?}", sim.esc.relay.status);
        assert_eq!(sim.peer_saved_parameters.len(), 2);
        assert_eq!(sim.peer_rejected_sources, 0);
    }
    #[test]
    fn full_supported_esc_image_flashes_from_actual_nor_staging_with_production_can() {
        let mut sim = Simulator::new().unwrap();
        sim.esc.flasher = controller_esc::firmware::Flasher::with_storage(Box::new(
            nor_storage::NorStorage::new(),
        ));
        sim.run(500, None).unwrap();
        let mut image = vec![0x5a; 256 * 1024];
        image[..2].copy_from_slice(&1u16.to_le_bytes());
        image[4..6].copy_from_slice(&256u16.to_le_bytes());
        image[6..8].copy_from_slice(&0x1234u16.to_le_bytes());
        assert!(sim.esc.flasher.begin(
            1,
            image.len() as u32,
            sim.esc.telemetry.esc_hardware_id,
            State::Disarmed,
            false,
            500
        ));
        for (offset, chunk) in image.chunks(512).enumerate() {
            assert_eq!(
                sim.esc.flasher.chunk((offset * 512) as u32, chunk, 500),
                Some((offset * 512) + chunk.len())
            );
        }
        assert!(sim.esc.flasher.end(500));
        sim.run(50000, None).unwrap();
        assert_eq!(sim.esc.flasher.code, 4, "{:?}", sim.esc.flasher.phase);
        assert_eq!(sim.esc.flasher.progress, 1000);
        assert!(sim.peer_flash_complete);
        assert_eq!(&sim.peer_flash[..image.len() - 32], &image[32..]);
        assert_eq!(sim.peer_rejected_sources, 0);
        assert_eq!(sim.controller.state, State::Disarmed);
    }
}

#[cfg(test)]
#[path = "../../firmware-sp140/src/diagnostic_json.rs"]
mod diagnostic_json;
