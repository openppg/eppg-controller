//! Legacy packed diagnostics ABI independent of MCU, NVS or JSON allocation.
pub const RECORD_SIZE: usize = 92;
pub const SNAPSHOT_SIZE: usize = 37;
pub const SNAPSHOT_MAGIC: u32 = 0x44484731;
pub fn get_u32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap_or([0; 4]))
}
pub fn put_u32(b: &mut [u8], at: usize, v: u32) {
    b[at..at + 4].copy_from_slice(&v.to_le_bytes());
}
pub fn resolve_reset(sdk: u8, raw0: u8, raw1: u8) -> u8 {
    if (1..=10).contains(&sdk) {
        sdk
    } else if raw0 == 21 || raw1 == 21 {
        11
    } else {
        0
    }
}
pub fn planned(value: u8) -> u8 {
    if value <= 2 {
        value
    } else {
        0
    }
}
pub fn build_record(
    boot: u32,
    sdk: u8,
    raw0: u8,
    raw1: u8,
    nvs_plan: u8,
    snapshot: &[u8; SNAPSHOT_SIZE],
) -> [u8; RECORD_SIZE] {
    let mut b = [0; RECORD_SIZE];
    put_u32(&mut b, 0, boot);
    b[28] = resolve_reset(sdk, raw0, raw1);
    b[29] = raw0;
    b[30] = raw1;
    b[32] = planned(nvs_plan);
    if get_u32(snapshot, 0) == SNAPSHOT_MAGIC {
        b[33] = 1;
        let uptime = get_u32(snapshot, 4);
        put_u32(&mut b, 4, uptime);
        for (to, from) in [(8, 12), (12, 16), (16, 20)] {
            put_u32(&mut b, to, uptime.saturating_sub(get_u32(snapshot, from)));
        }
        put_u32(&mut b, 20, get_u32(snapshot, 24));
        put_u32(&mut b, 24, get_u32(snapshot, 28));
        b[31] = if snapshot[32] <= 2 { snapshot[32] } else { 0 };
        b[34] = (snapshot[33] != 0) as u8;
        b[35] = (snapshot[34] != 0) as u8;
        if b[32] == 0 {
            b[32] = planned(snapshot[35]);
        }
    }
    b
}
#[derive(Clone, Debug)]
pub struct History {
    records: [[u8; RECORD_SIZE]; 16],
    len: usize,
}
impl Default for History {
    fn default() -> Self {
        Self {
            records: [[0; RECORD_SIZE]; 16],
            len: 0,
        }
    }
}
impl History {
    pub fn push(&mut self, record: [u8; RECORD_SIZE]) {
        if self.len == 16 {
            self.records.copy_within(1..16, 0);
            self.records[15] = record;
        } else {
            self.records[self.len] = record;
            self.len += 1;
        }
    }
    pub fn records(&self) -> &[[u8; RECORD_SIZE]] {
        &self.records[..self.len]
    }
    pub fn clear(&mut self) {
        self.len = 0;
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn packed_legacy_record_offsets_and_planned_fallback() {
        let mut s = [0; SNAPSHOT_SIZE];
        put_u32(&mut s, 0, SNAPSHOT_MAGIC);
        put_u32(&mut s, 4, 1000);
        put_u32(&mut s, 12, 990);
        put_u32(&mut s, 16, 800);
        put_u32(&mut s, 20, 1100);
        put_u32(&mut s, 24, 50000);
        put_u32(&mut s, 28, 42000);
        s[32] = 2;
        s[33] = 1;
        s[35] = 1;
        let r = build_record(10, 0, 21, 0, 0, &s);
        assert_eq!(&r[28..37], &[11, 21, 0, 2, 1, 1, 1, 0, 0]);
        assert_eq!(get_u32(&r, 8), 10);
        assert_eq!(get_u32(&r, 12), 200);
        assert_eq!(get_u32(&r, 16), 0);
        assert_eq!(get_u32(&r, 20), 50000);
        assert_eq!(get_u32(&r, 24), 42000);
        assert_eq!(r.len(), 92);
    }
    #[test]
    fn new_sdk_reset_values_are_not_reinterpreted_as_legacy_codes() {
        assert_eq!(resolve_reset(11, 0, 0), 0);
        assert_eq!(resolve_reset(11, 21, 0), 11);
        assert_eq!(resolve_reset(4, 21, 0), 4);
        assert_eq!(build_record(1, 0, 0, 0, 1, &[0; 37])[32], 1);
    }
    #[test]
    fn history_rollover_includes_current_boot() {
        let mut history = History::default();
        for boot in 1..=20 {
            let mut r = [0; 92];
            put_u32(&mut r, 0, boot);
            history.push(r);
        }
        assert_eq!(history.records().len(), 16);
        assert_eq!(get_u32(&history.records()[0], 0), 5);
        assert_eq!(get_u32(&history.records()[15], 0), 20);
        history.clear();
        assert!(history.records().is_empty());
    }
}
