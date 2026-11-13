//! FastLink v4 wire ABI. Explicit little-endian fields; no compiler layout casts.
#![allow(clippy::field_reassign_with_default)] // Linear field/offset correspondence is intentionally auditable.
pub const VERSION: u8 = 4;
pub const SIZE: usize = 230;
pub const NO_TEMP: i16 = i16::MIN;
#[allow(non_snake_case)] // Preserve firmware field names for wire-audit traceability.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FastLink {
    pub version: u8,
    pub packet_id: u32,
    pub uptime_ms: u32,
    pub altitude_cm: i32,
    pub baro_temp_dC: i16,
    pub baro_pressure_dHPa: u16,
    pub vario_cmps: i16,
    pub mcu_temp_dC: i16,
    pub pot_raw: u16,
    pub device_state: u8,
    pub esc_status: u8,
    pub esc_volts_dV: u16,
    pub esc_amps_dA: i16,
    pub esc_phase_current_dA: i16,
    pub esc_rpm: i32,
    pub esc_temp_mos_dC: i16,
    pub esc_temp_cap_dC: i16,
    pub esc_temp_mcu_dC: i16,
    pub esc_temp_motor_dC: i16,
    pub esc_inPWM: u16,
    pub esc_outPWM: u16,
    pub esc_v_modulation: u16,
    pub esc_error: u16,
    pub esc_selfcheck: u16,
    pub esc_hardware_id: u16,
    pub esc_fw_version: u16,
    pub esc_bootloader_version: u16,
    pub esc_runtime_ms: u32,
    pub esc_sn_code: [u8; 16],
    pub bms_status: u8,
    pub bms_soc: u8,
    pub bms_volts_dV: u16,
    pub bms_amps_dA: i16,
    pub bms_energy_cycle_mAh: u32,
    pub bms_battery_cycle: u32,
    pub bms_fail_level: u8,
    pub bms_is_charging: u8,
    pub bms_is_charge_mos: u8,
    pub bms_is_discharge_mos: u8,
    pub bms_charge_wire: u8,
    pub bms_low_soc_warning: u8,
    pub bms_battery_ready: u8,
    pub bms_highest_temp_C: i8,
    pub bms_lowest_temp_C: i8,
    pub bms_cell_max_mV: u16,
    pub bms_cell_min_mV: u16,
    pub bms_voltage_diff_mV: u16,
    pub bms_battery_id: [u8; 33],
    pub bms_type: u8,
    pub bms_cell_voltages_mV: [u16; 24],
    pub bms_temp_sensors_C: [i8; 8],
    pub esc_warning_alerts: u64,
    pub esc_critical_alerts: u64,
    pub bms_warning_alerts: u64,
    pub bms_critical_alerts: u64,
}
impl Default for FastLink {
    fn default() -> Self {
        Self {
            version: VERSION,
            packet_id: 0,
            uptime_ms: 0,
            altitude_cm: 0,
            baro_temp_dC: 0,
            baro_pressure_dHPa: 0,
            vario_cmps: 0,
            mcu_temp_dC: 0,
            pot_raw: 0,
            device_state: 0,
            esc_status: 0,
            esc_volts_dV: 0,
            esc_amps_dA: 0,
            esc_phase_current_dA: 0,
            esc_rpm: 0,
            esc_temp_mos_dC: 0,
            esc_temp_cap_dC: 0,
            esc_temp_mcu_dC: 0,
            esc_temp_motor_dC: 0,
            esc_inPWM: 0,
            esc_outPWM: 0,
            esc_v_modulation: 0,
            esc_error: 0,
            esc_selfcheck: 0,
            esc_hardware_id: 0,
            esc_fw_version: 0,
            esc_bootloader_version: 0,
            esc_runtime_ms: 0,
            esc_sn_code: [0; 16],
            bms_status: 0,
            bms_soc: 0,
            bms_volts_dV: 0,
            bms_amps_dA: 0,
            bms_energy_cycle_mAh: 0,
            bms_battery_cycle: 0,
            bms_fail_level: 0,
            bms_is_charging: 0,
            bms_is_charge_mos: 0,
            bms_is_discharge_mos: 0,
            bms_charge_wire: 0,
            bms_low_soc_warning: 0,
            bms_battery_ready: 0,
            bms_highest_temp_C: 0,
            bms_lowest_temp_C: 0,
            bms_cell_max_mV: 0,
            bms_cell_min_mV: 0,
            bms_voltage_diff_mV: 0,
            bms_battery_id: [0; 33],
            bms_type: 0,
            bms_cell_voltages_mV: [0; 24],
            bms_temp_sensors_C: [0; 8],
            esc_warning_alerts: 0,
            esc_critical_alerts: 0,
            bms_warning_alerts: 0,
            bms_critical_alerts: 0,
        }
    }
}
impl FastLink {
    pub fn encode(&self) -> [u8; SIZE] {
        let mut out = [0u8; SIZE];
        out[0..1].copy_from_slice(&self.version.to_le_bytes());
        out[1..5].copy_from_slice(&self.packet_id.to_le_bytes());
        out[5..9].copy_from_slice(&self.uptime_ms.to_le_bytes());
        out[9..13].copy_from_slice(&self.altitude_cm.to_le_bytes());
        out[13..15].copy_from_slice(&self.baro_temp_dC.to_le_bytes());
        out[15..17].copy_from_slice(&self.baro_pressure_dHPa.to_le_bytes());
        out[17..19].copy_from_slice(&self.vario_cmps.to_le_bytes());
        out[19..21].copy_from_slice(&self.mcu_temp_dC.to_le_bytes());
        out[21..23].copy_from_slice(&self.pot_raw.to_le_bytes());
        out[23..24].copy_from_slice(&self.device_state.to_le_bytes());
        out[24..25].copy_from_slice(&self.esc_status.to_le_bytes());
        out[25..27].copy_from_slice(&self.esc_volts_dV.to_le_bytes());
        out[27..29].copy_from_slice(&self.esc_amps_dA.to_le_bytes());
        out[29..31].copy_from_slice(&self.esc_phase_current_dA.to_le_bytes());
        out[31..35].copy_from_slice(&self.esc_rpm.to_le_bytes());
        out[35..37].copy_from_slice(&self.esc_temp_mos_dC.to_le_bytes());
        out[37..39].copy_from_slice(&self.esc_temp_cap_dC.to_le_bytes());
        out[39..41].copy_from_slice(&self.esc_temp_mcu_dC.to_le_bytes());
        out[41..43].copy_from_slice(&self.esc_temp_motor_dC.to_le_bytes());
        out[43..45].copy_from_slice(&self.esc_inPWM.to_le_bytes());
        out[45..47].copy_from_slice(&self.esc_outPWM.to_le_bytes());
        out[47..49].copy_from_slice(&self.esc_v_modulation.to_le_bytes());
        out[49..51].copy_from_slice(&self.esc_error.to_le_bytes());
        out[51..53].copy_from_slice(&self.esc_selfcheck.to_le_bytes());
        out[53..55].copy_from_slice(&self.esc_hardware_id.to_le_bytes());
        out[55..57].copy_from_slice(&self.esc_fw_version.to_le_bytes());
        out[57..59].copy_from_slice(&self.esc_bootloader_version.to_le_bytes());
        out[59..63].copy_from_slice(&self.esc_runtime_ms.to_le_bytes());
        for (i, v) in self.esc_sn_code.iter().enumerate() {
            out[63 + i..63 + (i + 1)].copy_from_slice(&v.to_le_bytes());
        }
        out[79..80].copy_from_slice(&self.bms_status.to_le_bytes());
        out[80..81].copy_from_slice(&self.bms_soc.to_le_bytes());
        out[81..83].copy_from_slice(&self.bms_volts_dV.to_le_bytes());
        out[83..85].copy_from_slice(&self.bms_amps_dA.to_le_bytes());
        out[85..89].copy_from_slice(&self.bms_energy_cycle_mAh.to_le_bytes());
        out[89..93].copy_from_slice(&self.bms_battery_cycle.to_le_bytes());
        out[93..94].copy_from_slice(&self.bms_fail_level.to_le_bytes());
        out[94..95].copy_from_slice(&self.bms_is_charging.to_le_bytes());
        out[95..96].copy_from_slice(&self.bms_is_charge_mos.to_le_bytes());
        out[96..97].copy_from_slice(&self.bms_is_discharge_mos.to_le_bytes());
        out[97..98].copy_from_slice(&self.bms_charge_wire.to_le_bytes());
        out[98..99].copy_from_slice(&self.bms_low_soc_warning.to_le_bytes());
        out[99..100].copy_from_slice(&self.bms_battery_ready.to_le_bytes());
        out[100..101].copy_from_slice(&self.bms_highest_temp_C.to_le_bytes());
        out[101..102].copy_from_slice(&self.bms_lowest_temp_C.to_le_bytes());
        out[102..104].copy_from_slice(&self.bms_cell_max_mV.to_le_bytes());
        out[104..106].copy_from_slice(&self.bms_cell_min_mV.to_le_bytes());
        out[106..108].copy_from_slice(&self.bms_voltage_diff_mV.to_le_bytes());
        for (i, v) in self.bms_battery_id.iter().enumerate() {
            out[108 + i..108 + (i + 1)].copy_from_slice(&v.to_le_bytes());
        }
        out[141..142].copy_from_slice(&self.bms_type.to_le_bytes());
        for (i, v) in self.bms_cell_voltages_mV.iter().enumerate() {
            out[142 + i * 2..142 + (i + 1) * 2].copy_from_slice(&v.to_le_bytes());
        }
        for (i, v) in self.bms_temp_sensors_C.iter().enumerate() {
            out[190 + i..190 + (i + 1)].copy_from_slice(&v.to_le_bytes());
        }
        out[198..206].copy_from_slice(&self.esc_warning_alerts.to_le_bytes());
        out[206..214].copy_from_slice(&self.esc_critical_alerts.to_le_bytes());
        out[214..222].copy_from_slice(&self.bms_warning_alerts.to_le_bytes());
        out[222..230].copy_from_slice(&self.bms_critical_alerts.to_le_bytes());
        out
    }
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() != SIZE || data[0] != VERSION {
            return None;
        }
        let mut value = Self::default();
        value.version = u8::from_le_bytes(data[0..1].try_into().ok()?);
        value.packet_id = u32::from_le_bytes(data[1..5].try_into().ok()?);
        value.uptime_ms = u32::from_le_bytes(data[5..9].try_into().ok()?);
        value.altitude_cm = i32::from_le_bytes(data[9..13].try_into().ok()?);
        value.baro_temp_dC = i16::from_le_bytes(data[13..15].try_into().ok()?);
        value.baro_pressure_dHPa = u16::from_le_bytes(data[15..17].try_into().ok()?);
        value.vario_cmps = i16::from_le_bytes(data[17..19].try_into().ok()?);
        value.mcu_temp_dC = i16::from_le_bytes(data[19..21].try_into().ok()?);
        value.pot_raw = u16::from_le_bytes(data[21..23].try_into().ok()?);
        value.device_state = u8::from_le_bytes(data[23..24].try_into().ok()?);
        value.esc_status = u8::from_le_bytes(data[24..25].try_into().ok()?);
        value.esc_volts_dV = u16::from_le_bytes(data[25..27].try_into().ok()?);
        value.esc_amps_dA = i16::from_le_bytes(data[27..29].try_into().ok()?);
        value.esc_phase_current_dA = i16::from_le_bytes(data[29..31].try_into().ok()?);
        value.esc_rpm = i32::from_le_bytes(data[31..35].try_into().ok()?);
        value.esc_temp_mos_dC = i16::from_le_bytes(data[35..37].try_into().ok()?);
        value.esc_temp_cap_dC = i16::from_le_bytes(data[37..39].try_into().ok()?);
        value.esc_temp_mcu_dC = i16::from_le_bytes(data[39..41].try_into().ok()?);
        value.esc_temp_motor_dC = i16::from_le_bytes(data[41..43].try_into().ok()?);
        value.esc_inPWM = u16::from_le_bytes(data[43..45].try_into().ok()?);
        value.esc_outPWM = u16::from_le_bytes(data[45..47].try_into().ok()?);
        value.esc_v_modulation = u16::from_le_bytes(data[47..49].try_into().ok()?);
        value.esc_error = u16::from_le_bytes(data[49..51].try_into().ok()?);
        value.esc_selfcheck = u16::from_le_bytes(data[51..53].try_into().ok()?);
        value.esc_hardware_id = u16::from_le_bytes(data[53..55].try_into().ok()?);
        value.esc_fw_version = u16::from_le_bytes(data[55..57].try_into().ok()?);
        value.esc_bootloader_version = u16::from_le_bytes(data[57..59].try_into().ok()?);
        value.esc_runtime_ms = u32::from_le_bytes(data[59..63].try_into().ok()?);
        for (i, v) in value.esc_sn_code.iter_mut().enumerate() {
            *v = u8::from_le_bytes(data[63 + i..63 + (i + 1)].try_into().ok()?);
        }
        value.bms_status = u8::from_le_bytes(data[79..80].try_into().ok()?);
        value.bms_soc = u8::from_le_bytes(data[80..81].try_into().ok()?);
        value.bms_volts_dV = u16::from_le_bytes(data[81..83].try_into().ok()?);
        value.bms_amps_dA = i16::from_le_bytes(data[83..85].try_into().ok()?);
        value.bms_energy_cycle_mAh = u32::from_le_bytes(data[85..89].try_into().ok()?);
        value.bms_battery_cycle = u32::from_le_bytes(data[89..93].try_into().ok()?);
        value.bms_fail_level = u8::from_le_bytes(data[93..94].try_into().ok()?);
        value.bms_is_charging = u8::from_le_bytes(data[94..95].try_into().ok()?);
        value.bms_is_charge_mos = u8::from_le_bytes(data[95..96].try_into().ok()?);
        value.bms_is_discharge_mos = u8::from_le_bytes(data[96..97].try_into().ok()?);
        value.bms_charge_wire = u8::from_le_bytes(data[97..98].try_into().ok()?);
        value.bms_low_soc_warning = u8::from_le_bytes(data[98..99].try_into().ok()?);
        value.bms_battery_ready = u8::from_le_bytes(data[99..100].try_into().ok()?);
        value.bms_highest_temp_C = i8::from_le_bytes(data[100..101].try_into().ok()?);
        value.bms_lowest_temp_C = i8::from_le_bytes(data[101..102].try_into().ok()?);
        value.bms_cell_max_mV = u16::from_le_bytes(data[102..104].try_into().ok()?);
        value.bms_cell_min_mV = u16::from_le_bytes(data[104..106].try_into().ok()?);
        value.bms_voltage_diff_mV = u16::from_le_bytes(data[106..108].try_into().ok()?);
        for (i, v) in value.bms_battery_id.iter_mut().enumerate() {
            *v = u8::from_le_bytes(data[108 + i..108 + (i + 1)].try_into().ok()?);
        }
        value.bms_type = u8::from_le_bytes(data[141..142].try_into().ok()?);
        for (i, v) in value.bms_cell_voltages_mV.iter_mut().enumerate() {
            *v = u16::from_le_bytes(data[142 + i * 2..142 + (i + 1) * 2].try_into().ok()?);
        }
        for (i, v) in value.bms_temp_sensors_C.iter_mut().enumerate() {
            *v = i8::from_le_bytes(data[190 + i..190 + (i + 1)].try_into().ok()?);
        }
        value.esc_warning_alerts = u64::from_le_bytes(data[198..206].try_into().ok()?);
        value.esc_critical_alerts = u64::from_le_bytes(data[206..214].try_into().ok()?);
        value.bms_warning_alerts = u64::from_le_bytes(data[214..222].try_into().ok()?);
        value.bms_critical_alerts = u64::from_le_bytes(data[222..230].try_into().ok()?);
        Some(value)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wire_prefix_masks_and_round_trip() {
        let mut f = FastLink::default();
        f.packet_id = 0x12345678;
        f.esc_warning_alerts = 0x8877665544332211;
        f.bms_critical_alerts = 1 << 63;
        f.esc_temp_motor_dC = NO_TEMP;
        f.bms_cell_voltages_mV = [4200; 24];
        let b = f.encode();
        assert_eq!(&b[1..5], &0x12345678u32.to_le_bytes());
        assert_eq!(&b[198..206], &0x8877665544332211u64.to_le_bytes());
        assert_eq!(&b[222..230], &(1u64 << 63).to_le_bytes());
        assert_eq!(FastLink::decode(&b), Some(f));
        assert!(FastLink::decode(&b[..229]).is_none());
    }
}
