//! ANT BMS CAN parser, matching pinned fd54852. No SPI/Arduino dependencies.
use crate::fastlink::FastLink;
#[derive(Clone, Debug)]
pub struct AntBms {
    pub data: FastLink,
    pub temps: [f32; 8],
    pub highest_temp: f32,
    pub lowest_temp: f32,
    last_message: Option<u32>,
    coherent: bool,
    endian_detected: bool,
    little_endian: bool,
    temp_seen: bool,
}
impl Default for AntBms {
    fn default() -> Self {
        Self {
            data: FastLink::default(),
            temps: [f32::NAN; 8],
            highest_temp: f32::NAN,
            lowest_temp: f32::NAN,
            last_message: None,
            coherent: false,
            endian_detected: false,
            little_endian: false,
            temp_seen: false,
        }
    }
}
fn le(d: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([d[i], d[i + 1]])
}
fn temp(v: u8) -> f32 {
    if v == 0 {
        f32::NAN
    } else {
        v as f32 - 40.
    }
}
impl AntBms {
    pub fn packet(&mut self, id: u32, d: &[u8], now: u32) -> bool {
        if d.len() != 8 {
            return false;
        }
        // The reference updates lastMessageTime for every received CAN packet.
        self.last_message = Some(now);
        match id {
            0x18ff28f4 => {
                self.data.bms_charge_wire = d[0] & 1;
                self.data.bms_is_charging = (d[0] >> 1) & 1;
                self.data.bms_low_soc_warning = (d[0] >> 2) & 1;
                self.data.bms_battery_ready = (d[0] >> 3) & 1;
                self.data.bms_is_discharge_mos = (d[0] >> 4) & 1;
                self.data.bms_is_charge_mos = (d[0] >> 5) & 1;
                self.data.bms_soc = d[1];
                self.data.bms_volts_dV = le(d, 4);
                self.data.bms_amps_dA =
                    (le(d, 2) as i32 - 5000).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
                self.data.bms_fail_level = d[6];
            }
            0x18fe28f4 => {
                self.data.bms_cell_max_mV = le(d, 0);
                self.data.bms_cell_min_mV = le(d, 2);
                self.data.bms_voltage_diff_mV = le(d, 0).saturating_sub(le(d, 2));
                self.highest_temp = temp(d[4]);
                self.lowest_temp = temp(d[5]);
            }
            0x18a328f4 => {
                self.data.bms_battery_cycle =
                    u32::from_be_bytes(d[..4].try_into().unwrap_or([0; 4]));
                self.data.bms_energy_cycle_mAh =
                    u32::from_be_bytes(d[4..8].try_into().unwrap_or([0; 4]));
            }
            0x18a028f4 | 0x18a128f4 | 0x18a228f4 => {
                let offset = (((id >> 16) & 255) - 0xa0) as usize * 8;
                self.data.bms_battery_id[offset..offset + 8].copy_from_slice(d);
                if offset == 0 {
                    self.data.bms_type = 2;
                }
            }
            0x18b428f4 => {
                self.highest_temp = f32::NAN;
                self.lowest_temp = f32::NAN;
                for (i, b) in d.iter().enumerate() {
                    let t = temp(*b);
                    self.temps[i] = t;
                    if !t.is_nan() {
                        self.temp_seen = true;
                        if self.highest_temp.is_nan() || t > self.highest_temp {
                            self.highest_temp = t;
                        }
                        if self.lowest_temp.is_nan() || t < self.lowest_temp {
                            self.lowest_temp = t;
                        }
                    }
                }
            }
            _ => {
                let pf = (id >> 16) & 255;
                if id >> 24 == 0x18 && (id >> 8) & 255 == 0x28 && (0xc8..0xce).contains(&pf) {
                    if !self.endian_detected && self.data.bms_cell_max_mV > 0 {
                        let reference = self.data.bms_cell_max_mV as f32 / 1000.;
                        let mut le_score = 0.;
                        let mut be_score = 0.;
                        for i in 0..4 {
                            let a = le(d, i * 2) as f32 / 1000.;
                            let b = u16::from_be_bytes([d[i * 2], d[i * 2 + 1]]) as f32 / 1000.;
                            if (2.0..=4.5).contains(&a) {
                                le_score += 1. - (a - reference).abs();
                            }
                            if (2.0..=4.5).contains(&b) {
                                be_score += 1. - (b - reference).abs();
                            }
                        }
                        self.little_endian = le_score > be_score;
                        self.endian_detected = true;
                        self.data.bms_type = if self.little_endian { 2 } else { 1 };
                    }
                    for i in 0..4 {
                        self.data.bms_cell_voltages_mV[(pf as usize - 0xc8) * 4 + i] =
                            if self.little_endian {
                                le(d, i * 2)
                            } else {
                                u16::from_be_bytes([d[i * 2], d[i * 2 + 1]])
                            };
                    }
                } else {
                    return false;
                }
            }
        }
        self.coherent |= self.data.bms_volts_dV > 50 && self.data.bms_cell_max_mV > 500;
        true
    }
    pub fn connected(&self, now: u32) -> bool {
        self.coherent
            && self
                .last_message
                .map(|at| now.wrapping_sub(at) < 500)
                .unwrap_or(false)
    }
    /// Apply the latched disconnected-probe policy before threshold monitoring.
    pub fn temperatures(&self) -> [f32; 6] {
        let mut out = [
            self.temps[0],
            self.temps[1],
            self.temps[2],
            self.temps[3],
            self.temps[4],
            self.temps[5],
        ];
        if self.temp_seen {
            let mut ignored = 0;
            for value in &mut out[2..] {
                if value.is_nan() {
                    if ignored < 2 {
                        ignored += 1;
                    } else {
                        *value = -40.;
                    }
                }
            }
        }
        out
    }
    pub fn snapshot(&self, now: u32) -> FastLink {
        let mut f = self.data.clone();
        f.bms_status = if self.connected(now) { 1 } else { 0 };
        f.bms_highest_temp_C = self.highest_temp as i8;
        f.bms_lowest_temp_C = self.lowest_temp as i8;
        for (i, t) in self.temperatures().iter().enumerate() {
            f.bms_temp_sensors_C[i] = *t as i8;
        }
        f
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn complete_battery_identity_exact_ids_only() {
        let mut b = AntBms::default();
        for (i, id) in [0x18a028f4, 0x18a128f4, 0x18a228f4].into_iter().enumerate() {
            assert!(b.packet(id, &[i as u8 + 1; 8], 0));
        }
        assert_eq!(&b.data.bms_battery_id[..8], &[1; 8]);
        assert_eq!(&b.data.bms_battery_id[8..16], &[2; 8]);
        assert_eq!(&b.data.bms_battery_id[16..24], &[3; 8]);
        assert!(!b.packet(0x18a128f5, &[9; 8], 0));
        assert_eq!(&b.data.bms_battery_id[8..16], &[2; 8]);
    }
    #[test]
    fn coherent_gate_latches_and_fault_keeps_connection() {
        let mut b = AntBms::default();
        b.packet(0x18ff28f4, &[0x38, 90, 0x88, 0x13, 0xa0, 3, 0, 0], 100);
        assert!(!b.connected(100));
        b.packet(0x18fe28f4, &[0x68, 0x10, 0x68, 0x10, 70, 60, 0, 0], 110);
        assert!(b.connected(110));
        b.packet(0x18fe28f4, &[0x68, 0x10, 0, 0, 70, 60, 0, 0], 200);
        assert!(b.connected(200));
        assert!(!b.connected(700));
    }
    #[test]
    fn detect_both_endian_variants() {
        for little in [false, true] {
            let mut b = AntBms::default();
            b.packet(0x18fe28f4, &[0x68, 0x10, 0x04, 0x10, 70, 60, 0, 0], 100);
            let cell = if little {
                4200u16.to_le_bytes()
            } else {
                4200u16.to_be_bytes()
            };
            let mut frame = [0; 8];
            for chunk in frame.as_chunks_mut::<2>().0 {
                chunk.copy_from_slice(&cell);
            }
            b.packet(0x18c828f4, &frame, 110);
            assert_eq!(b.data.bms_type, if little { 2 } else { 1 });
            assert_eq!(b.data.bms_cell_voltages_mV[0], 4200);
        }
    }
    #[test]
    fn probe_policy_remains_latched() {
        let mut b = AntBms::default();
        b.packet(0x18b428f4, &[60, 60, 65, 65, 65, 65, 0, 0], 10);
        b.packet(0x18b428f4, &[0; 8], 20);
        let t = b.temperatures();
        assert!(t[2].is_nan());
        assert!(t[3].is_nan());
        assert_eq!(t[4], -40.);
        assert_eq!(t[5], -40.);
    }
}
