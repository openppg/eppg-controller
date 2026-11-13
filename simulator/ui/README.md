# OpenPPG Hardware Lab

This React console is a thin control surface over the native controller and
LVGL emulators. It does not reimplement controller state or screen widgets in
JavaScript.

From the repository root:

```bash
./simulator/run.sh --test-only
./test/test_screenshots/build_and_run.sh --local
cd simulator/ui
npm install
npm start
```

Then open `http://127.0.0.1:4174/`.

- Drag the throttle to send a 0–4095 ADC value through the production filter.
- Click the controller button once, then hold it for two seconds to arm.
- Hold the same button for two seconds while armed to toggle cruise.
- Use **Inject Telemetry** to set exact ESC/BMS link and sensor values. They
  cross the virtual CAN wire and vendor decoders before reaching firmware UI.
- Use fault injection to drop/corrupt frames or trigger bus-off recovery.

The controller display is the native production LVGL framebuffer at its real
160×128 resolution. Light mode is the default.
