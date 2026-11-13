//! Fixed-capacity threshold/connection monitors with original SensorID ordinals.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Level {
    #[default]
    Ok,
    WarnLow,
    WarnHigh,
    CritLow,
    CritHigh,
    Info,
}
impl Level {
    pub fn critical(self) -> bool {
        matches!(self, Self::CritLow | Self::CritHigh)
    }
    pub fn warning(self) -> bool {
        matches!(self, Self::WarnLow | Self::WarnHigh)
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Thresholds {
    pub warn_low: f32,
    pub warn_high: f32,
    pub crit_low: f32,
    pub crit_high: f32,
    pub hysteresis: f32,
}
impl Thresholds {
    pub const fn new(wl: f32, wh: f32, cl: f32, ch: f32, h: f32) -> Self {
        Self {
            warn_low: wl,
            warn_high: wh,
            crit_low: cl,
            crit_high: ch,
            hysteresis: h,
        }
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Sensor {
    pub level: Level,
}
impl Sensor {
    pub fn check(&mut self, v: f32, t: Thresholds, hysteresis: bool) -> Option<Level> {
        let old = self.level;
        let next = if v.is_nan() {
            Level::Ok
        } else if !hysteresis || old == Level::Ok {
            if v <= t.crit_low {
                Level::CritLow
            } else if v <= t.warn_low {
                Level::WarnLow
            } else if v >= t.crit_high {
                Level::CritHigh
            } else if v >= t.warn_high {
                Level::WarnHigh
            } else {
                Level::Ok
            }
        } else {
            match old {
                Level::WarnLow => {
                    if v <= t.crit_low {
                        Level::CritLow
                    } else if v > t.warn_low + t.hysteresis {
                        Level::Ok
                    } else {
                        old
                    }
                }
                Level::WarnHigh => {
                    if v >= t.crit_high {
                        Level::CritHigh
                    } else if v < t.warn_high - t.hysteresis {
                        Level::Ok
                    } else {
                        old
                    }
                }
                Level::CritLow => {
                    if v > t.crit_low + t.hysteresis {
                        if v <= t.warn_low + t.hysteresis {
                            Level::WarnLow
                        } else {
                            Level::Ok
                        }
                    } else {
                        old
                    }
                }
                Level::CritHigh => {
                    if v < t.crit_high - t.hysteresis {
                        if v >= t.warn_high - t.hysteresis {
                            Level::WarnHigh
                        } else {
                            Level::Ok
                        }
                    } else {
                        old
                    }
                }
                _ => old,
            }
        };
        self.level = next;
        if next != old {
            Some(next)
        } else {
            None
        }
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct LinkMonitor {
    established: bool,
    up_since: Option<u32>,
    lost_since: Option<u32>,
    pub level: Level,
}
impl LinkMonitor {
    pub fn check(&mut self, now: u32, connected: bool, armed: bool) -> Option<Level> {
        let old = self.level;
        if connected {
            self.lost_since = None;
            let since = *self.up_since.get_or_insert(now);
            if now.wrapping_sub(since) >= 1000 {
                self.established = true;
                self.level = Level::Ok;
            }
        } else {
            self.up_since = None;
            if self.established {
                let since = *self.lost_since.get_or_insert(now);
                if now.wrapping_sub(since) >= 500 {
                    self.level = if armed {
                        Level::CritHigh
                    } else {
                        Level::WarnHigh
                    };
                }
            }
        }
        if self.level != old {
            Some(self.level)
        } else {
            None
        }
    }
}
#[derive(Clone, Debug)]
pub struct Monitors {
    pub levels: [Level; 44],
    sensors: [Sensor; 44],
    links: [LinkMonitor; 2],
    bms_seen: bool,
    bms_grace: Option<u32>,
    last_esc: bool,
    last_bms: bool,
}
impl Default for Monitors {
    fn default() -> Self {
        Self {
            levels: [Level::Ok; 44],
            sensors: [Sensor::default(); 44],
            links: [LinkMonitor::default(); 2],
            bms_seen: false,
            bms_grace: None,
            last_esc: false,
            last_bms: false,
        }
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Data {
    pub esc_connected: bool,
    pub bms_connected: bool,
    pub armed: bool,
    pub esc_initialized: bool,
    pub bms_initialized: bool,
    pub baro_initialized: bool,
    pub esc_temps: [f32; 4],
    pub running_errors: u16,
    pub selfcheck_errors: u16,
    pub bms_temps: [f32; 6],
    pub pack_voltage: f32,
    pub max_cell: f32,
    pub min_cell: f32,
    pub soc: f32,
    pub voltage_diff: f32,
    pub charge_mos: bool,
    pub discharge_mos: bool,
    pub baro_temp: f32,
    pub cpu_temp: f32,
}
impl Monitors {
    fn numeric(&mut self, id: usize, v: f32, t: Thresholds, h: bool) {
        self.sensors[id].check(v, t, h);
        self.levels[id] = self.sensors[id].level;
    }
    fn boolean(&mut self, id: usize, active: bool, level: Level) {
        self.levels[id] = if active { level } else { Level::Ok };
    }
    pub fn update(&mut self, now: u32, d: Data) {
        self.links[0].check(now, d.esc_connected, d.armed);
        self.levels[25] = self.links[0].level;
        self.links[1].check(now, d.bms_connected, d.armed);
        self.levels[40] = self.links[1].level;
        if !d.esc_connected {
            for id in 0..25 {
                self.levels[id] = Level::Ok;
                self.sensors[id] = Sensor::default();
            }
        } else {
            if !self.last_esc {
                self.sensors[..25].fill(Sensor::default());
            }
            self.boolean(24, !d.esc_initialized, Level::CritHigh);
            let ts = [
                Thresholds::new(-10., 90., -20., 110., 1.),
                Thresholds::new(-10., 80., -20., 95., 1.),
                Thresholds::new(-10., 85., -20., 100., 1.),
                Thresholds::new(-20., 105., -25., 115., 1.),
            ];
            for (id, t) in ts.into_iter().enumerate() {
                self.numeric(id, d.esc_temps[id], t, true);
            }
            for (id, bit) in [(4, 0), (5, 1), (6, 2), (7, 6), (8, 7)] {
                self.boolean(id, d.running_errors & (1 << bit) != 0, Level::CritHigh);
            }
            for (id, bit) in [
                (10, 0),
                (11, 1),
                (12, 2),
                (13, 3),
                (14, 4),
                (15, 5),
                (16, 6),
                (17, 7),
                (18, 8),
                (19, 9),
                (20, 10),
                (21, 11),
                (22, 13),
                (23, 14),
            ] {
                self.boolean(id, d.selfcheck_errors & (1 << bit) != 0, Level::CritHigh);
            }
        }
        if d.bms_connected && !self.bms_seen {
            self.bms_seen = true;
            self.bms_grace = Some(now);
        }
        let grace = self
            .bms_grace
            .map(|at| now.wrapping_sub(at) < 2000)
            .unwrap_or(false);
        if !d.bms_connected || grace {
            for id in 26..40 {
                self.levels[id] = Level::Ok;
                self.sensors[id] = Sensor::default();
            }
        } else {
            if !self.last_bms {
                self.sensors[26..40].fill(Sensor::default());
            }
            for (id, value) in d.bms_temps.iter().enumerate() {
                self.numeric(
                    26 + id,
                    *value,
                    Thresholds::new(-10., 50., -15., if id < 2 { 60. } else { 56. }, 1.),
                    true,
                );
            }
            if d.pack_voltage > 5. {
                for (id, v, t) in [
                    (32, d.max_cell, Thresholds::new(-1., 4.19, -1., 4.2, 0.)),
                    (33, d.min_cell, Thresholds::new(3.2, 4.5, 3., 4.8, 0.)),
                    (34, d.soc, Thresholds::new(15., 101., 5., 110., 0.)),
                    (
                        35,
                        d.pack_voltage,
                        Thresholds::new(79.2, 100.4, 69.6, 100.8, 0.),
                    ),
                    (36, d.voltage_diff, Thresholds::new(-1., 0.2, -2., 0.4, 0.)),
                ] {
                    self.numeric(id, v, t, false);
                }
                // Charge-MOS monitor is disabled in the baseline.
                self.levels[37] = Level::Ok;
                self.boolean(38, !d.discharge_mos, Level::CritHigh);
            } else {
                self.levels[32..39].fill(Level::Ok);
                self.sensors[32..39].fill(Sensor::default());
            }
            self.boolean(39, !d.bms_initialized, Level::WarnHigh);
        }
        self.numeric(
            41,
            d.baro_temp,
            Thresholds::new(0., 50., -10., 80., 1.),
            false,
        );
        self.boolean(42, !d.baro_initialized, Level::WarnHigh);
        self.numeric(
            43,
            d.cpu_temp,
            Thresholds::new(0., 60., -10., 80., 1.),
            true,
        );
        self.last_esc = d.esc_connected;
        self.last_bms = d.bms_connected;
    }
    pub fn masks(&self) -> [u64; 4] {
        let mut out = [0u64; 4];
        for (id, level) in self.levels.iter().enumerate().take(41) {
            let group = if id < 26 { 0 } else { 2 };
            let bit = if id < 26 { id } else { id - 26 };
            if level.critical() {
                out[group + 1] |= 1 << bit;
            } else if level.warning() {
                out[group] |= 1 << bit;
            }
        }
        out
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hysteresis() {
        let mut s = Sensor::default();
        let t = Thresholds::new(-10., 90., -20., 110., 1.);
        assert_eq!(s.check(90., t, true), Some(Level::WarnHigh));
        assert_eq!(s.check(89.5, t, true), None);
        assert_eq!(s.check(110., t, true), Some(Level::CritHigh));
        assert_eq!(s.check(f32::NAN, t, true), Some(Level::Ok));
    }
    #[test]
    fn link_debounce_and_severity() {
        let mut l = LinkMonitor::default();
        l.check(0, true, false);
        l.check(1000, true, false);
        l.check(1100, false, false);
        assert_eq!(l.check(1599, false, true), None);
        assert_eq!(l.check(1600, false, true), Some(Level::CritHigh));
        assert_eq!(l.check(1700, false, false), Some(Level::WarnHigh));
        l.check(1800, true, false);
        assert_eq!(l.check(2800, true, false), Some(Level::Ok));
    }
}
