# Local controller hardware lab

Build either native backend described in [the simulator README](../README.md),
then run:

```sh
npm ci
npm start
```

Open http://127.0.0.1:4174/. The server chooses the built Rust production-core
simulator when available, otherwise the native C++ reference. Explicit selection:

```sh
EPPG_SIM_BACKEND=rust npm start
EPPG_SIM_BACKEND=native npm start
```

The Rust backend renders the same Rust LVGL UI as the firmware. Native C++ uses
the standalone production framebuffer renderer built under
`build-host-screenshots/`. These are live rendered160x128 pixels, not prerecorded
screen images. Throttle, arm/cruise gestures, sensor injection and wire faults
run in deterministic virtual time. RF and electrical behavior remain outside
this functional simulator; the separate ESP32-S3 QEMU path executes the actual
firmware instructions and models unsupported peripheral boundaries.

`npm test` runs a local API/real wire-fault/framebuffer integration test once a
backend is built. It does not require Vite/React to exercise the backend API.
`npm run dev` starts API and Vite separately for UI development.
