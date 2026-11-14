//! Device settings shared by firmware, UI and host simulator. Persistence lives in adapters.
#[derive(Clone, Debug)]
pub struct Settings {
    pub version_major: u8,
    pub version_minor: u8,
    pub screen_rotation: u8,
    pub sea_pressure: f32,
    pub metric_temp: bool,
    pub metric_alt: bool,
    pub performance_mode: u8,
    pub theme: u8,
    pub armed_time: u16,
    pub revision: u8,
    pub timezone_offset: i32,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            version_major: 8,
            version_minor: 1,
            screen_rotation: 3,
            sea_pressure: 1013.25,
            metric_temp: true,
            metric_alt: true,
            performance_mode: 0,
            theme: 0,
            armed_time: 0,
            revision: 0,
            timezone_offset: 0,
        }
    }
}
impl Settings {
    pub fn sanitize(&mut self) {
        if !matches!(self.screen_rotation, 1 | 3) {
            self.screen_rotation = 3;
        }
        if !self.sea_pressure.is_finite() || self.sea_pressure < 300. || self.sea_pressure > 1200. {
            self.sea_pressure = 1013.25;
        }
        if self.performance_mode > 1 {
            self.performance_mode = 0;
        }
        if self.theme > 1 {
            self.theme = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn invalid_pressure_cannot_poison_altitude_and_vario() {
        for sea_pressure in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0., 299., 1201.] {
            let mut settings = Settings {
                sea_pressure,
                ..Settings::default()
            };
            settings.sanitize();
            assert_eq!(settings.sea_pressure, 1013.25);
        }
        for sea_pressure in [300., 1013.25, 1200.] {
            let mut settings = Settings {
                sea_pressure,
                ..Settings::default()
            };
            settings.sanitize();
            assert_eq!(settings.sea_pressure, sea_pressure);
        }
    }
}
