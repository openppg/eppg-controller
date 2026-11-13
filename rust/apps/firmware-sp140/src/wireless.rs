use crate::{
    app::{Commands, Request, Shared},
    platform,
    storage::RadioSettings,
};
use controller_core::{Role, State};
use controller_protocols::{
    fastlink::FastLink,
    wireless::{Ack, Control},
};
use esp_idf_svc::{
    espnow::{EspNow, PeerInfo},
    eventloop::EspSystemEventLoop,
    hal::peripherals::Peripherals,
    sys,
    wifi::{ClientConfiguration, Configuration, EspWifi},
};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{sync_channel, SyncSender},
    Arc,
};
#[derive(Clone, Copy)]
pub struct RemoteControl {
    pub captured_at: u32,
    pub raw: u16,
    pub state: State,
    pub mode: u8,
}
#[cfg(feature = "qemu")]
pub fn spawn(
    _settings: RadioSettings,
    _shared: Shared,
    _commands: Commands,
) -> SyncSender<RemoteControl> {
    let (tx, rx) = sync_channel(4);
    platform::console(
        "EPPG_QEMU ESP-NOW RF unsupported; encrypted transport modeled by native Rust lab\n",
    );
    crate::spawn_task(move || while rx.recv().is_ok() {});
    tx
}
#[cfg(not(feature = "qemu"))]
pub fn spawn(
    settings: RadioSettings,
    shared: Shared,
    commands: Commands,
) -> SyncSender<RemoteControl> {
    let (tx, rx) = sync_channel::<RemoteControl>(4);
    std::thread::Builder::new()
        .name("espnow".into())
        .stack_size(6000)
        .spawn(move || {
            if settings.role == Role::Standalone
                || settings.peer.is_none()
                || settings.secret.is_none()
            {
                return;
            }
            let peer = settings.peer.unwrap();
            let secret = settings.secret.unwrap();
            let peripherals = Peripherals::take().expect("Wi-Fi peripheral singleton");
            let eventloop = EspSystemEventLoop::take().expect("Wi-Fi event loop");
            let mut wifi = EspWifi::new(peripherals.modem, eventloop, None).expect("Wi-Fi driver");
            wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))
                .expect("Wi-Fi STA mode");
            wifi.start().expect("Wi-Fi STA start");
            unsafe {
                sys::esp_wifi_set_channel(
                    settings.channel,
                    sys::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE,
                );
                sys::esp_wifi_set_ps(sys::wifi_ps_type_t_WIFI_PS_NONE);
            }
            let radio = EspNow::take().expect("ESP-NOW init");
            let pmk = secret.map(|v| v ^ 0xa5);
            radio.set_pmk(&pmk).expect("ESP-NOW PMK");
            let info = PeerInfo {
                peer_addr: peer,
                lmk: secret.map(|v| v ^ 0x5a),
                channel: settings.channel,
                ifidx: sys::wifi_interface_t_WIFI_IF_STA,
                encrypt: true,
                ..Default::default()
            };
            radio.add_peer(info).expect("encrypted ESP-NOW peer");
            let session = unsafe { sys::esp_random() }.max(1);
            let role = settings.role;
            let target = commands.clone();
            radio
                .register_recv_cb(move |info, data| {
                    if info.src_addr != &peer {
                        return;
                    }
                    let now = platform::now();
                    let request = if role == Role::Receiver {
                        Control::decode(data).map(Request::Control)
                    } else if let Some(ack) = Ack::decode(data) {
                        if ack.session == session {
                            Some(Request::Ack(ack))
                        } else {
                            None
                        }
                    } else {
                        FastLink::decode(data).map(Request::Mirrored)
                    };
                    if let Some(r) = request {
                        let _ = target.try_send(r);
                    }
                    let _ = now;
                })
                .expect("ESP-NOW receive callback");
            let in_flight = Arc::new(AtomicBool::new(false));
            let sent = in_flight.clone();
            radio
                .register_send_cb(move |_, _| {
                    sent.store(false, Ordering::Release);
                })
                .expect("ESP-NOW send callback");
            let mut publisher = controller_protocols::wireless::RemotePublisher::new(session);
            let mut last_ack = 0;
            let mut last_telemetry = 0;
            loop {
                let now = platform::now();
                while let Ok(value) = rx.try_recv() {
                    publisher.enqueue(controller_protocols::wireless::FreshControl {
                        captured_at: value.captured_at,
                        throttle: value.raw,
                        state: value.state as u8,
                        mode: value.mode,
                    });
                }
                if !in_flight.load(Ordering::Acquire) {
                    let bytes = if role == Role::Remote {
                        publisher.take(now).map(|p| p.encode().to_vec())
                    } else {
                        let s = shared.lock().unwrap().clone();
                        if s.accepted_session != 0 && now.wrapping_sub(last_ack) >= 40 {
                            last_ack = now;
                            Some(
                                Ack {
                                    session: s.accepted_session,
                                    sequence: s.accepted_sequence,
                                    uptime: now,
                                    state: s.state as u8,
                                    flags: Ack::ACCEPTED
                                        | if s.control_age <= 150 {
                                            Ack::LINK_READY
                                        } else {
                                            0
                                        }
                                        | if s.failsafe { Ack::FAILSAFE } else { 0 },
                                    age: s.control_age.min(65535) as u16,
                                }
                                .encode()
                                .to_vec(),
                            )
                        } else if now.wrapping_sub(last_telemetry) >= 100 {
                            last_telemetry = now;
                            Some(s.telemetry.encode().to_vec())
                        } else {
                            None
                        }
                    };
                    if let Some(b) = bytes {
                        in_flight.store(true, Ordering::Release);
                        if radio.send(peer, &b).is_err() {
                            in_flight.store(false, Ordering::Release);
                        }
                    }
                }
                platform::sleep(20);
            }
        })
        .expect("wireless task");
    tx
}
