# Pinned legacy CAN reference oracle

These are **test-only original implementations**, retained unchanged so native
simulation and differential tests can compare the migrated code against the
previous firmware. They are not linked into production ESP32-S3 firmware.
Their includes are satisfied by local host transport/time stubs, not an Arduino
SDK installation.

- `sine/`: `CanardAdapter` and `SineEsc`, SINE-ESC-CAN revision
  `2ab56a4e5b52e4456317c8ee3e3d802b232c6148`.
- `ant/`: `BMS_CAN`, ANT-BMS-CAN revision
  `fd54852bc6f1c9608e37af9ca7c13ea4135c095b`.

Licenses are included in each directory. The plain C libcanard and generated
DSDL headers/codecs are shared from the pinned production
`components/dronecan/src` tree. For the pre-migration baseline, supply
`EPPG_FIRMWARE_DEPS` or `DRONECAN_DIR` explicitly. No implicit network fetch or
user-specific absolute path is used.
