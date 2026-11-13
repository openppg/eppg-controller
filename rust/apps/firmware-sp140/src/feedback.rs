use crate::{app::Shared, platform};
use controller_core::Effects;
use std::{
    collections::VecDeque,
    sync::mpsc::{sync_channel, SyncSender},
};
#[derive(Clone, Copy)]
pub enum Feedback {
    Effects(Effects),
    Vibrate(u16, u8),
    Warning,
}
pub fn spawn(shared: Shared) -> SyncSender<Feedback> {
    let (tx, rx) = sync_channel(10);
    std::thread::Builder::new()
        .name("audio-haptics".into())
        .stack_size(3000)
        .spawn(move || {
            let mut tones: VecDeque<(u16, u16)> = VecDeque::new();
            let mut vibes: VecDeque<(u16, u8)> = VecDeque::new();
            let (mut tone_at, mut tone_ms, mut vibe_at, mut vibe_ms) = (0u32, 0u32, 0u32, 0u32);
            let mut last_led = 0;
            let mut led = false;
            loop {
                let now = platform::now();
                while let Ok(f) = rx.try_recv() {
                    match f {
                        Feedback::Vibrate(ms, intensity) => {
                            if vibes.len() < 5 {
                                vibes.push_back((ms, intensity));
                            }
                        }
                        Feedback::Warning => {
                            if vibes.len() + 4 <= 5 {
                                vibes.extend([(100, 0), (150, 255), (150, 0), (150, 255)]);
                            }
                        }
                        Feedback::Effects(e) => {
                            if tones.len() < 10 {
                                if e.contains(Effects::ARM) {
                                    tones.extend([(1047, 200), (0, 80), (1568, 200)]);
                                } else if e.contains(Effects::DISARM) {
                                    tones.extend([(1568, 200), (0, 80), (1047, 200)]);
                                } else if e.contains(Effects::ARM_FAIL) {
                                    tones.push_back((1760, 200));
                                } else if e.contains(Effects::MODE_CHANGED) {
                                    tones.push_back((1976, 200));
                                }
                            }
                            if e.0
                                & (Effects::ARM
                                    | Effects::DISARM
                                    | Effects::ARM_FAIL
                                    | Effects::CRUISE_START
                                    | Effects::CRUISE_END
                                    | Effects::PAIRING)
                                != 0
                                && vibes.len() < 5
                            {
                                vibes.push_back((400, 255));
                            }
                        }
                    }
                }
                if tone_ms == 0 || now.wrapping_sub(tone_at) >= tone_ms {
                    if let Some((hz, ms)) = tones.pop_front() {
                        unsafe {
                            if hz != 0 {
                                platform::eppg_pwm_frequency(1, hz as u32);
                            }
                            platform::eppg_pwm_write(1, if hz == 0 { 0 } else { 128 });
                        }
                        tone_at = now;
                        tone_ms = ms as u32;
                    } else {
                        unsafe {
                            platform::eppg_pwm_write(1, 0);
                        }
                        tone_ms = 0;
                    }
                }
                if vibe_ms == 0 || now.wrapping_sub(vibe_at) >= vibe_ms {
                    if let Some((ms, intensity)) = vibes.pop_front() {
                        unsafe {
                            platform::eppg_pwm_write(0, intensity as u32);
                        }
                        vibe_at = now;
                        vibe_ms = ms as u32;
                    } else {
                        unsafe {
                            platform::eppg_pwm_write(0, 0);
                        }
                        vibe_ms = 0;
                    }
                }
                if now.wrapping_sub(last_led) >= 500 {
                    last_led = now;
                    let s = shared.lock().unwrap();
                    led = !led;
                    let color = if s.state != controller_core::State::Disarmed || led {
                        0x00ff00
                    } else {
                        0
                    };
                    unsafe {
                        platform::eppg_pixel_write(21, color);
                    }
                }
                platform::sleep(5);
            }
        })
        .expect("audio/haptic task");
    tx
}
