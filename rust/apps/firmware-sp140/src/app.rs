use controller_core::monitor::Level;
use controller_core::{Role, Settings, State};
use controller_protocols::{
    fastlink::FastLink,
    wireless::{Ack, Control},
};
use std::sync::{mpsc::SyncSender, Arc, Mutex};
pub type Shared = Arc<Mutex<Snapshot>>;
#[derive(Clone)]
pub struct Snapshot {
    pub bms_temperatures: [f32; 6],
    pub settings: Settings,
    pub telemetry: FastLink,
    pub levels: [Level; 44],
    pub state: State,
    pub accepted_session: u32,
    pub accepted_sequence: u32,
    pub control_age: u32,
    pub failsafe: bool,
    pub maintenance_pending: bool,
    pub ota: bool,
    pub relay: bool,
    pub transfer_quiet: bool,
    pub pairing: bool,
    pub connected: bool,
    pub esc_status: Vec<u8>,
    pub esc_result: Vec<u8>,
    pub esc_fw_status: [u8; 4],
    pub last_throttle: u32,
    pub last_ui: u32,
    pub last_bms: u32,
    pub esc_initialized: bool,
    pub bms_initialized: bool,
    pub baro_initialized: bool,
    pub baro_temperature: f32,
    pub cpu_temperature: f32,
}
pub struct UiSnapshot {
    pub bms_temperatures: [f32; 6],
    pub settings: Settings,
    pub telemetry: FastLink,
    pub levels: [Level; 44],
    pub connected: bool,
    pub pairing: bool,
}
impl Snapshot {
    pub fn ui_snapshot(&self) -> UiSnapshot {
        UiSnapshot {
            bms_temperatures: self.bms_temperatures,
            settings: self.settings.clone(),
            telemetry: self.telemetry.clone(),
            levels: self.levels,
            connected: self.connected,
            pairing: self.pairing,
        }
    }
    pub fn new(settings: Settings, _role: Role) -> Self {
        Self {
            bms_temperatures: [f32::NAN; 6],
            settings,
            telemetry: FastLink::default(),
            levels: [Level::Ok; 44],
            state: State::Disarmed,
            accepted_session: 0,
            accepted_sequence: 0,
            control_age: u32::MAX,
            failsafe: true,
            maintenance_pending: false,
            ota: false,
            relay: false,
            transfer_quiet: false,
            pairing: false,
            connected: false,
            esc_status: vec![0; 7],
            esc_result: vec![],
            esc_fw_status: [0; 4],
            last_throttle: 0,
            last_ui: 0,
            last_bms: 0,
            esc_initialized: false,
            bms_initialized: false,
            baro_initialized: false,
            baro_temperature: f32::MIN_POSITIVE,
            cpu_temperature: 0.,
        }
    }
}
#[derive(Clone, Debug)]
pub enum Request {
    CruisePwm(u16),
    Settings(Settings),
    Control(Control),
    Ack(Ack),
    Mirrored(FastLink),
    HardwareInfo,
    Relay(u8, Vec<u8>),
    FlashBegin(u16, u32),
    FlashReject(u8),
    FlashProgress(u32, usize),
    FlashEnd,
    FlashAbort,
}
pub type Commands = SyncSender<Request>;
