# ESP-NOW Wireless Controller

The same SP140 controller firmware supports three persisted roles:

- `disabled`: existing standalone wired-controller behavior (the default)
- `remote`: handheld controller; reads its potentiometer/button and never initializes local ESC/BMS CAN
- `receiver`: power-unit controller; receives control over ESP-NOW and remains the sole owner of ESC/BMS CAN

The receiver's existing BLE telemetry, configuration, and OTA services are unchanged. In parallel, it mirrors the same Fast-Link telemetry packet to the remote at 10 Hz. Control packets are sent by the remote at 50 Hz, and the receiver returns application acknowledgements at 25 Hz.

## Provisioning a pair

Provision over USB while both controllers are disarmed. Both units must use the same channel and random 16-byte pairing secret. Each unit stores the other unit's Wi-Fi station MAC address.

1. Flash both controllers and request their settings:

   ```json
   {"command":"sync"}
   ```

   Record `wireless_mac` from each response.

2. Generate one random 32-hex-character key. For example, on a development machine:

   ```sh
   openssl rand -hex 16
   ```

3. Configure the CAN-connected controller, substituting the remote's MAC:

   ```json
   {"wireless":{"role":"receiver","peer":"AA:BB:CC:DD:EE:FF","channel":6,"key":"00112233445566778899aabbccddeeff"}}
   ```

4. Configure the handheld controller, substituting the receiver's MAC and using the identical channel/key:

   ```json
   {"wireless":{"role":"remote","peer":"11:22:33:44:55:66","channel":6,"key":"00112233445566778899aabbccddeeff"}}
   ```

5. Reboot both controllers. A successful boot prints an `[ESPNOW]` line containing the role, channel, and local MAC.

To return a controller to normal wired operation:

```json
{"wireless":{"role":"disabled"}}
```

Reboot after every role, peer, key, or channel change. The pairing key is intentionally omitted from sync responses.

## Safety behavior

- The receiver starts disarmed and requires a fresh explicit `DISARMED` command before it will accept an arm transition.
- A new remote session is admitted only through a disarmed packet. Reboots never restore an armed state.
- The receiver validates source MAC, ESP-NOW CCMP encryption, protocol version, packet CRC, ranges, session, and monotonic sequence.
- If no fresh control packet arrives for 150 ms, the throttle task immediately changes to `DISARMED` and sends idle on its current 20 ms control cycle. Slow NVS, sound, vibration, and UI side effects run later on a non-control task.
- The receiver is authoritative. Arm rejection, cruise override, and failsafe state are reflected back to the remote through application acknowledgements.
- ESP-NOW callbacks only validate/copy packets; they never operate CAN or perform state-transition side effects.

## Required hardware validation

This is flight-critical code and is not flight-ready based on compilation or unit tests alone. Test with the propeller removed before any ground or flight testing:

- turn the remote off at several throttle positions;
- shield or move either antenna until the link is lost;
- reboot each controller independently while the other remains powered;
- introduce BLE app connections and sustained 50 Hz BLE telemetry;
- test stale, duplicated, corrupted, and wrong-peer packets;
- verify the receiver never re-arms after any outage without a new physical arm sequence;
- measure throttle-loop jitter and actual idle-command latency under RF interference;
- range-test with production enclosures, pilot body shadowing, battery/ESC current, and final antenna orientation.

