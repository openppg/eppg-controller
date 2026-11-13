//! PWM arithmetic ported from throttle.cpp; integer divisions intentionally truncate.
pub const MIN: u16 = 1035;
pub const MAX: u16 = 1950;
pub const IDLE: u16 = 1000;
pub const CRUISE_MAX: u16 = 1680;
pub fn map_mode(raw: u16, mode: u8) -> u16 {
    let max = if mode == 0 { 1770 } else { 1950 };
    (1035 + (raw as u32 * (max - 1035)) / 4095) as u16
}
pub fn map_full(raw: u16) -> u16 {
    map_mode(raw, 1)
}
pub fn limited(current: i32, last: i32, threshold: i32) -> i32 {
    let delta = current - last;
    let limit = if delta > 0 { threshold } else { threshold * 2 };
    last + delta.clamp(-limit, limit)
}
pub fn ramp(value: u16, last: u16, mode: u8) -> u16 {
    limited(value as i32, last as i32, if mode == 0 { 8 } else { 27 })
        .clamp(1035, if mode == 0 { 1770 } else { 1950 }) as u16
}
pub fn cruise_output_allowed(pwm: u16) -> bool {
    pwm <= CRUISE_MAX
}
#[derive(Clone, Debug)]
pub struct Filter {
    samples: [u16; 8],
    len: usize,
    next: usize,
}
impl Filter {
    pub const fn new() -> Self {
        Self {
            samples: [0; 8],
            len: 0,
            next: 0,
        }
    }
    pub fn clear(&mut self) {
        self.len = 0;
        self.next = 0;
    }
    pub fn reset(&mut self, value: u16) {
        self.samples = [value; 8];
        self.len = 8;
        self.next = 0;
    }
    pub fn push(&mut self, value: u16) {
        self.samples[self.next] = value;
        self.next = (self.next + 1) % 8;
        self.len = (self.len + 1).min(8);
    }
    pub fn average(&self) -> u16 {
        if self.len == 0 {
            0
        } else {
            (self.samples[..self.len]
                .iter()
                .map(|v| *v as u32)
                .sum::<u32>()
                / self.len as u32) as u16
        }
    }
}
impl Default for Filter {
    fn default() -> Self {
        Self::new()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mapping_exhaustive() {
        for raw in 0..=4095 {
            assert_eq!(map_mode(raw, 0), 1035 + (raw as u32 * 735 / 4095) as u16);
            assert_eq!(map_full(raw), 1035 + (raw as u32 * 915 / 4095) as u16);
        }
    }
    #[test]
    fn ramp_matches() {
        assert_eq!(ramp(1950, 1035, 1), 1062);
        assert_eq!(ramp(1035, 1600, 0), 1584);
        assert_eq!(ramp(1950, 1950, 0), 1770);
    }
    #[test]
    fn filter_wrap() {
        let mut f = Filter::new();
        assert_eq!(f.average(), 0);
        for n in 1..=9 {
            f.push(n);
        }
        assert_eq!(f.average(), 5);
        f.reset(1234);
        assert_eq!(f.average(), 1234);
        f.clear();
        assert_eq!(f.average(), 0);
    }
}
