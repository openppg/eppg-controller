# Fast-Link telemetry v4

Fast-Link v4 extends the v3 payload with the controller's active ESC and BMS
monitor alerts. Bytes 0–197 are unchanged from v3, including the raw device
fields `esc_error`, `esc_selfcheck`, and `bms_fail_level`.

The packed payload is 230 bytes. All multibyte integers use ESP32/native little
endian encoding.

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | `uint8` | Protocol version (`4`) |
| 198 | `uint64` | `esc_warning_alerts` |
| 206 | `uint64` | `esc_critical_alerts` |
| 214 | `uint64` | `bms_warning_alerts` |
| 222 | `uint64` | `bms_critical_alerts` |

An alert is active when its bit is set. Warning and critical masks are mutually
exclusive for a given alert at a given instant. The firmware may move an alert
between masks as its severity changes, such as a CAN link-loss alert when the
controller is armed or disarmed.

## ESC alert bits

| Bit | Alert |
| ---: | --- |
| 0 | ESC MOS temperature |
| 1 | ESC MCU temperature |
| 2 | ESC capacitor temperature |
| 3 | Motor temperature |
| 4 | Over-current |
| 5 | Locked rotor |
| 6 | Over-temperature |
| 7 | Over-voltage |
| 8 | Voltage drop |
| 9 | Throttle saturation |
| 10 | Motor current output self-check |
| 11 | Total current output self-check |
| 12 | Motor voltage output self-check |
| 13 | Capacitor NTC self-check |
| 14 | MOS NTC self-check |
| 15 | Bus voltage range self-check |
| 16 | Bus voltage sample self-check |
| 17 | Motor impedance too low |
| 18 | Motor impedance too high |
| 19 | Motor voltage detector 1 |
| 20 | Motor voltage detector 2 |
| 21 | Motor current detector 2 |
| 22 | Software/hardware incompatibility |
| 23 | Unsupported bootloader |
| 24 | Controller TWAI initialization failure |
| 25 | Established ESC CAN link lost |

## BMS alert bits

| Bit | Alert |
| ---: | --- |
| 0 | BMS MOS temperature |
| 1 | BMS balance-resistor temperature |
| 2 | BMS T1 temperature |
| 3 | BMS T2 temperature |
| 4 | BMS T3 temperature |
| 5 | BMS T4 temperature |
| 6 | Highest cell voltage |
| 7 | Lowest cell voltage |
| 8 | State of charge |
| 9 | Total pack voltage |
| 10 | Cell-voltage differential |
| 11 | Charge MOS state |
| 12 | Discharge MOS state |
| 13 | Controller BMS CAN initialization failure |
| 14 | Established BMS CAN link lost |

For recording, store the four alert masks alongside the three raw device error
fields. The masks report controller-evaluated conditions; the raw fields retain
the exact error values sent by the ESC and BMS.
