#include "sp140/debug.h"

static unsigned long last_update = 0;
static float fake_values[4] = {0.0, 0.0, 0.0, 0.0}; // For smooth transitions
static const float UPDATE_INTERVAL = 100; // Update every 100ms

// Helper function to generate smooth oscillating values
float oscillate(float& current_value, float min, float max, float step) {
    static bool increasing = true;

    if (increasing) {
        current_value += step;
        if (current_value >= max) {
            increasing = false;
            current_value = max;
        }
    } else {
        current_value -= step;
        if (current_value <= min) {
            increasing = true;
            current_value = min;
        }
    }
    return current_value;
}

void generateFakeTelemetry(STR_ESC_TELEMETRY_140& escTelemetry,
                          STR_BMS_TELEMETRY_140& bmsTelemetry,
                          UnifiedBatteryData& unifiedBatteryData,
                          float& altitude) {
    unsigned long now = millis();
    if (now - last_update < UPDATE_INTERVAL) return;
    last_update = now;

    #ifdef SCREEN_DEBUG_STATIC
        // Static maximum values for UI design
        escTelemetry.volts = 100.0;
        escTelemetry.amps = 50.0;
        escTelemetry.mos_temp = 70.0;
    #else
        // Oscillating values
        escTelemetry.volts = oscillate(fake_values[0], 65.0, 100.0, 0.1);
        escTelemetry.amps = oscillate(fake_values[1], 0.0, 50.0, 0.5);
        escTelemetry.mos_temp = oscillate(fake_values[2], 30.0, 70.0, 0.2);
    #endif

    escTelemetry.cap_temp = escTelemetry.mos_temp - 5;
    escTelemetry.mcu_temp = escTelemetry.mos_temp - 10;
    escTelemetry.eRPM = map(escTelemetry.amps * 100, 0, 5000, 0, 25000);
    escTelemetry.inPWM = map(escTelemetry.amps * 100, 0, 5000, 1000, 2000);
    escTelemetry.outPWM = escTelemetry.inPWM;
    escTelemetry.statusFlag = 0;
    escTelemetry.lastUpdateMs = now;

    // BMS Telemetry
    bmsTelemetry.soc = map(escTelemetry.volts * 100, 8000, 10000, 0, 100);
    bmsTelemetry.battery_voltage = escTelemetry.volts;
    bmsTelemetry.battery_current = escTelemetry.amps;
    bmsTelemetry.power = bmsTelemetry.battery_voltage * bmsTelemetry.battery_current / 1000.0; // kW
    bmsTelemetry.highest_cell_voltage = escTelemetry.volts / 24.0 + 0.1;
    bmsTelemetry.lowest_cell_voltage = escTelemetry.volts / 24.0 - 0.1;
    bmsTelemetry.highest_temperature = escTelemetry.mos_temp;
    bmsTelemetry.lowest_temperature = escTelemetry.mcu_temp;
    bmsTelemetry.energy_cycle = 3.5;
    bmsTelemetry.battery_cycle = 42;
    bmsTelemetry.battery_failure_level = 0;
    bmsTelemetry.voltage_differential = 0.2;
    bmsTelemetry.lastUpdateMs = now;

    // Unified Battery Data
    unifiedBatteryData.volts = bmsTelemetry.battery_voltage;
    unifiedBatteryData.amps = bmsTelemetry.battery_current;
    unifiedBatteryData.soc = bmsTelemetry.soc;

    // Altitude
    altitude = oscillate(fake_values[3], 0.0, 100.0, 0.5);
}