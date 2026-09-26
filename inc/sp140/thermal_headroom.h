#ifndef INC_SP140_THERMAL_HEADROOM_H_
#define INC_SP140_THERMAL_HEADROOM_H_

// Thermal headroom (issue #57): the highest power a pilot could hold for a
// sustained climb before a part reaches its warning or critical temperature.
// It caps the green best-climb band and adds yellow (would reach warning) and
// red (would reach critical) zones to the power bar.
//
// Pure C++ (no Arduino/FreeRTOS), like climb_efficiency.
//
// Model, per part (motor, ESC MOSFET / capacitor / MCU, battery cells):
//
//     dT/dt = (T_ss(P) - T) / tau,   T_ss(P) = T_base + k * P^2
//
// Heat goes with P^2 (resistive losses), cooling toward a baseline. Holding
// power P for H seconds from temperature T gives
//
//     T(H) = T_ss(P) + (T - T_ss(P)) * exp(-H / tau)
//
// so staying under a threshold Th needs
//
//     P <= sqrt(((Th - T e) / (1 - e) - T_base) / k),   e = exp(-H / tau).
//
// The projection always starts from the *measured* temperatures (a pack warm
// off the charger, a hot day, a motor still warm from the last flight), and
// the curves are learned from this flight. Fleet data (30 real SP140 flights,
// tools/hc_emulator/analysis/fleet_thermal.py) only provides the starting
// shape until the flight has shown its own:
//
//  - motor/ESC: where the part sits on its curve (T_base, which follows the
//    air temperature and cooling - the motor's is ~air + 21 C) and how steep
//    the curve is (k) are both fitted live, recursive least squares on each
//    10 s step with forgetting, from steps with the motor working (not idle
//    on the ground, where there is no prop airflow). The fleet fit is the
//    prior; tau stays at the fleet value (the part's thermal mass vs.
//    cooling).
//  - battery cells, BMS MOSFET, BMS balance: inside the pack, a static
//    thermal mass with no active cooling, so they only heat, near-linearly in
//    time at a given power (T(H) = T + rate * P^2 * H). The rate depends on
//    the pack type: at the same power a ~2.3 kWh pack's cells heat ~2.9x
//    faster than a ~4.6 kWh pack's (fewer cells in parallel), its BMS MOSFET
//    ~1.7x and balance sensor ~1.9x. The pack size is measured in flight from
//    SOC drop vs. energy (the app's setting is often wrong); each rate then
//    blends from that pack type's curve toward this flight's own heating.
//
// Parts covered: every temperature with an alert in the firmware that heats
// with throttle - motor, ESC MOSFET / capacitor / MCU, battery cells (hottest
// of T1-T4), BMS MOSFET, BMS balance. (The hand controller's CPU and baro
// temperatures are not driven by throttle and keep their own alerts.)
//
// Nothing is shown from priors alone: a part only limits the bar once this
// flight has taught it something - motor/ESC once minLearnS of powered data
// and a power change (climb vs. cruise) have pinned the heating steepness to
// learnedGainVarFrac of its prior uncertainty; pack parts once the pack size
// is measured and they have carried heatOnlyLearnKw2S of load.
//
// H is how long the climb could actually last: until the pack reaches
// socReservePct at that power (pack size learned from SOC drop vs. energy
// used), capped at horizonMaxS - a long climb, 600-900 m.
//
// Validated on the fleet (leave-one-flight-out priors, live learning on):
// 5-min forecasts are off by 3.4 C (motor) and 1.7-3.1 C (ESC) on average
// with little bias; all 4 warning crossings in the logs (2 motor, 2 battery)
// showed as yellow/red 40-190 s before they happened. For a 5-min climb the
// limit is ~7.5 kW median (motor on most flights, a warm battery on hot days).

#include <stdint.h>

enum ThermalPart : uint8_t {
  THERMAL_MOTOR = 0,
  THERMAL_ESC_MOS,
  THERMAL_ESC_CAP,
  THERMAL_ESC_MCU,
  THERMAL_BATTERY,       // hottest cell probe (T1-T4)
  THERMAL_BMS_MOS,
  THERMAL_BMS_BALANCE,
  THERMAL_PART_COUNT
};

struct ThermalPartModel {
  float warnC;         // warning threshold (end of the green zone)
  float critC;         // critical threshold (end of the yellow zone)
  float gainCPerKw2;   // k: steady-state rise per kW^2
  float tauS;          // time constant
  bool trackBase;      // estimate T_base in flight (false: heating only)
  float baseOverStartC;  // prior T_base = first reading + this (fleet)
  float packExponent;  // heating-only gain ~ (packRef / pack)^this; 0: none
};

struct ThermalHeadroomConfig {
  // Thresholds mirror inc/sp140/monitor_config.h (main.cpp copies the live
  // ones in); gains and time constants are fleet medians.
  ThermalPartModel parts[THERMAL_PART_COUNT] = {
    {105.0f, 115.0f, 1.00f, 207.0f, true, 21.0f, 0.0f},  // motor
    {90.0f, 110.0f, 0.55f, 179.0f, true, 5.0f, 0.0f},    // ESC MOSFET
    {85.0f, 100.0f, 0.61f, 198.0f, true, 5.0f, 0.0f},    // ESC capacitor
    {80.0f, 95.0f, 0.75f, 286.0f, true, 0.0f, 0.0f},     // ESC MCU
    // Pack parts, heating only: k = fleet rate * tau at packRefKwh
    // (rates on ~4.6 kWh packs: cells 1.37e-4, BMS MOSFET 1.42e-4,
    // balance 1.29e-4 C per kW^2 s).
    {50.0f, 56.0f, 0.685f, 5000.0f, false, 0.0f, 1.5f},   // battery cells
    {50.0f, 60.0f, 0.71f, 5000.0f, false, 0.0f, 0.75f},   // BMS MOSFET
    {50.0f, 60.0f, 0.645f, 5000.0f, false, 0.0f, 0.95f},  // BMS balance
  };
  uint32_t stepMs = 10000;        // observer / projection update period
  float horizonMaxS = 300.0f;     // longest sustained climb considered
  float thresholdMarginC = 3.0f;  // covers the model's cool bias
  float socReservePct = 15.0f;    // a climb ends here (SOC warning level)
  // Until measured, assume the small pack: it heats fastest (the safe side
  // for the battery; the climb horizon is capped anyway).
  float packKwhDefault = 2.6f;
  float minSocDropForPackPct = 5.0f;
  float packRefKwh = 4.6f;        // pack-part gains are given at this size
  // Pack-part heating learned in flight: weight of the observed rate is
  // E / (E + heatOnlyPriorKw2S) with E = integral of P^2 since the part's
  // first reading (1e4 kW^2 s ~ 10 min at 4 kW).
  float heatOnlyPriorKw2S = 10000.0f;
  // Live motor/ESC fit: prior spread, forgetting per step (0.995 at 10 s ~
  // a 30 min memory), temperature noise, and how far k may move.
  float basePriorSdC = 8.0f;
  float gainPriorRelSd = 0.5f;
  float rlsForgetting = 0.995f;
  float tempNoiseC = 0.35f;
  float gainMinRel = 0.25f;
  float gainMaxRel = 4.0f;
  // "Learned from this flight" gates for showing a part's limits. Only steps
  // with the motor working (>= minLearnPowerKw) teach the motor/ESC fits.
  float minLearnPowerKw = 1.5f;
  float minLearnS = 120.0f;
  float learnedGainVarFrac = 0.5f;
  float heatOnlyLearnKw2S = 3000.0f;
  float limitSmoothingS = 20.0f;  // displayed limits filter
  float maxPowerKw = 16.0f;       // limits above this mean "no limit"
};

struct ThermalHeadroomResult {
  bool valid;               // at least one part has learned from this flight
  uint8_t learnedMask;      // bit per ThermalPart that is learned
  float warnPowerKw;        // hold <= this: no warnings (maxPowerKw = none)
  float critPowerKw;        // hold <= this: no criticals (maxPowerKw = none)
  int8_t warnLimiter;       // ThermalPart that sets warnPowerKw, -1 none
  int8_t critLimiter;
  float horizonS;           // horizon used at the warn limit
  float packKwh;            // learned (or default) pack energy
  bool packLearned;         // packKwh measured this flight
  float batteryGain;        // effective battery-cell k in use (C per kW^2)
  float gainC[THERMAL_PART_COUNT];  // k in use per part (live fit / pack)
  float baseC[THERMAL_PART_COUNT];  // tracked T_base per part (NaN if none)
};

class ThermalHeadroom {
 public:
  ThermalHeadroom();
  explicit ThermalHeadroom(const ThermalHeadroomConfig& config);

  void setConfig(const ThermalHeadroomConfig& config);
  const ThermalHeadroomConfig& config() const { return config_; }
  void reset();

  // Feed at any rate (the UI task calls it ~30 Hz). tempsC: current part
  // temperatures, NaN when a sensor is missing. socPct NaN if unknown.
  void update(uint32_t nowMs, float powerKw,
              const float tempsC[THERMAL_PART_COUNT], float socPct);

  const ThermalHeadroomResult& result() const { return result_; }

  // Highest power holdable under `thresholdC` of `part` for horizonS
  // (0 if already over, maxPowerKw if unbounded). Exposed for tests/tools.
  float partLimit(int part, float thresholdC, float horizonS) const;

  // Heating gain in use for a part (pack-scaled / learned for the battery).
  float partGain(int part) const;

 private:
  void step(float dtS, float meanP2);
  float horizonFor(float powerKw) const;
  void startFit(int part, float firstTempC);
  void updateFit(int part, float prevC, float tempC, float dtS, float meanP2);
  bool partLearned(int part) const;
  float limitAll(bool critical, int8_t* limiter, float* horizonOut) const;

  ThermalHeadroomConfig config_;
  ThermalHeadroomResult result_;

  bool started_ = false;
  uint32_t stepStartMs_ = 0;
  float sumP2_ = 0.0f;
  uint32_t samples_ = 0;
  float lastTemp_[THERMAL_PART_COUNT];
  float prevTemp_[THERMAL_PART_COUNT];   // at the previous step
  // Live fit per part: theta = (T_base, k), covariance (p00, p01, p11).
  bool fitStarted_[THERMAL_PART_COUNT];
  float fitSeconds_[THERMAL_PART_COUNT];  // data time since the fit started
  float base_[THERMAL_PART_COUNT];
  float gain_[THERMAL_PART_COUNT];
  float p00_[THERMAL_PART_COUNT];
  float p01_[THERMAL_PART_COUNT];
  float p11_[THERMAL_PART_COUNT];

  // Heating-only parts: first reading and integral of P^2 (kW^2 s) since.
  float heatStartC_[THERMAL_PART_COUNT];
  float heatP2S_[THERMAL_PART_COUNT];

  float socStart_ = -1.0f;
  float energyKwh_ = 0.0f;
  float lastSoc_ = -1.0f;
  uint32_t lastMs_ = 0;
};

#endif  // INC_SP140_THERMAL_HEADROOM_H_
