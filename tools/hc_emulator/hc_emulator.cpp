// Interactive hand-controller emulator.
//
// Runs the real LVGL main screen (src/sp140/lvgl) and the climb efficiency
// estimator on a virtual clock. It is driven by tools/hc_emulator/server.py
// over stdin/stdout, one command per line:
//
//   init theme=<0|1> metric=<0|1> perf=<0|1>   rebuild the main screen
//   bar v=<0|1>                                climb-efficiency power bar on/off
//   mock level=<kW> margin=<kW> lo=<kW> hi=<kW> warn=<kW> crit=<kW>
//                                              show these bar values instead of
//                                              the estimators'; mock off=1
//   cfg window=<ms> settle=<ms> cv=<f> minp=<kW> minclimb=<m/s> minalt=<m> learn=<ms>
//       band=<f> binw=<kW> binhalflife=<n> minbinw=<f> q=<1/kW> maxp=<kW>
//       climbev=<m/s> minclimbw=<n> levelband=<m/s> priorslope=<m/s/kW>
//       levelhalflife=<n> minlevelw=<n>
//                                              estimator config (resets it)
//   reset                                      forget the learned curve
//   s dt=<ms> alt=<m> p=<kW> soc=<%> v=<V> armed=<0|1> cruise=<0|1>
//     bt=<C> et=<C> mt=<C> bms=<0|1> esc=<0|1>  one sample (the UI task tick)
//   render                                     reply: one JSON line, then
//                                              160*128 RGB565 LE pixels
//   quit
//
// Unknown keys are ignored and omitted keys keep their previous value, so the
// driver only has to send what changed.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "emulator_display.h"
#include "sp140/climb_efficiency.h"
#include "sp140/thermal_headroom.h"
#include "sp140/lvgl/lvgl_climb_efficiency.h"
#include "sp140/lvgl/lvgl_main_screen.h"
#include "sp140/lvgl/lvgl_updates.h"
#include "sp140/structs.h"

unsigned long eppgVirtualMillis = 1;  // 0 means "never armed" in the UI code

namespace {

typedef std::map<std::string, double> Args;

struct EmulatorState {
  STR_DEVICE_DATA_140_V1 deviceData = {};
  STR_ESC_TELEMETRY_140 esc = {};
  STR_BMS_TELEMETRY_140 bms = {};
  UnifiedBatteryData battery = {};
  float altitude = 0.0f;
  bool armed = false;
  bool cruising = false;
  unsigned int armedAtMillis = 0;
  unsigned long lastRenderMillis = 1;
  ClimbEfficiencyEstimator estimator;
  ThermalHeadroom thermal;
  bool mockActive = false;
  ClimbEfficiencyResult mock = {};
  float mockWarnKw = NAN;
  float mockCritKw = NAN;
};

EmulatorState state;

Args parseArgs(char* line) {
  Args args;
  for (char* tok = strtok(line, " \t\r\n"); tok; tok = strtok(NULL, " \t\r\n")) {
    char* eq = strchr(tok, '=');
    if (!eq) continue;
    *eq = '\0';
    args[tok] = strtod(eq + 1, NULL);
  }
  return args;
}

bool get(const Args& args, const char* key, double* out) {
  Args::const_iterator it = args.find(key);
  if (it == args.end()) return false;
  *out = it->second;
  return true;
}

void setFloat(const Args& args, const char* key, float* field) {
  double v;
  if (get(args, key, &v)) *field = static_cast<float>(v);
}

void setU32(const Args& args, const char* key, uint32_t* field) {
  double v;
  if (get(args, key, &v) && v >= 0) *field = static_cast<uint32_t>(v);
}

void setupScreen() {
  const bool dark = state.deviceData.theme == 1;
  emulator_teardown();
  emulator_init_display(dark);
  setupMainScreen(dark);
}

void cmdInit(const Args& args) {
  double v;
  STR_DEVICE_DATA_140_V1& dd = state.deviceData;
  if (get(args, "theme", &v)) dd.theme = static_cast<uint8_t>(v);
  if (get(args, "metric", &v)) dd.metric_alt = v != 0;
  if (get(args, "perf", &v)) dd.performance_mode = static_cast<uint8_t>(v);
  setupScreen();
}

void cmdConfig(const Args& args) {
  ClimbEfficiencyConfig cfg = state.estimator.config();
  setU32(args, "window", &cfg.windowMs);
  setU32(args, "settle", &cfg.settleMs);
  setU32(args, "learn", &cfg.learnEveryMs);
  setFloat(args, "cv", &cfg.maxPowerCv);
  setFloat(args, "minp", &cfg.minPowerKw);
  setFloat(args, "minclimb", &cfg.minClimbMs);
  setFloat(args, "minalt", &cfg.minAltitudeM);
  setFloat(args, "binhalflife", &cfg.binHalfLife);
  setFloat(args, "q", &cfg.curvatureQ);
  setFloat(args, "maxp", &cfg.maxPowerKw);
  setFloat(args, "climbev", &cfg.climbEvidenceMs);
  setFloat(args, "minclimbw", &cfg.minClimbWeight);
  setFloat(args, "band", &cfg.bandFraction);
  setFloat(args, "binw", &cfg.binWidthKw);
  setFloat(args, "minbinw", &cfg.minBinWeight);
  setFloat(args, "levelband", &cfg.levelBandMs);
  setFloat(args, "priorslope", &cfg.priorSlope);
  setFloat(args, "levelhalflife", &cfg.levelHalfLife);
  setFloat(args, "minlevelw", &cfg.minLevelWeight);
  setFloat(args, "minlevelp", &cfg.minLevelPowerKw);
  setFloat(args, "margink", &cfg.levelMarginK);
  setFloat(args, "marginfloor", &cfg.levelMarginFloorKw);
  setFloat(args, "maxmargin", &cfg.maxShownMarginKw);
  state.estimator.setConfig(cfg);
}

// One UI-task tick: mirrors refreshDisplay() in main.cpp.
void cmdSample(const Args& args) {
  double v;
  if (get(args, "dt", &v) && v > 0) eppgVirtualMillis += static_cast<unsigned long>(v);

  setFloat(args, "alt", &state.altitude);
  setFloat(args, "p", &state.battery.power);
  setFloat(args, "soc", &state.battery.soc);
  setFloat(args, "v", &state.battery.volts);
  state.battery.amps = state.battery.volts > 1.0f
      ? state.battery.power * 1000.0f / state.battery.volts : 0.0f;

  STR_BMS_TELEMETRY_140& bms = state.bms;
  STR_ESC_TELEMETRY_140& esc = state.esc;
  if (get(args, "bms", &v)) {
    bms.bmsState = v != 0 ? TelemetryState::CONNECTED : TelemetryState::NOT_CONNECTED;
  }
  if (get(args, "esc", &v)) {
    esc.escState = v != 0 ? TelemetryState::CONNECTED : TelemetryState::NOT_CONNECTED;
  }
  bms.soc = state.battery.soc;
  bms.battery_voltage = state.battery.volts;
  bms.battery_current = state.battery.amps;
  bms.power = state.battery.power;
  bms.lowest_cell_voltage = state.battery.volts / 24.0f;
  bms.highest_cell_voltage = bms.lowest_cell_voltage + 0.02f;
  setFloat(args, "bt", &bms.t1_temperature);
  bms.t2_temperature = bms.t1_temperature - 1.0f;
  bms.t3_temperature = NAN;
  bms.t4_temperature = NAN;
  esc.volts = state.battery.volts;
  esc.amps = state.battery.amps;
  setFloat(args, "et", &esc.mos_temp);
  esc.cap_temp = esc.mos_temp - 3.0f;
  setFloat(args, "ec", &esc.cap_temp);
  setFloat(args, "em", &esc.mcu_temp);
  setFloat(args, "mt", &esc.motor_temp);
  setFloat(args, "bm", &bms.mos_temperature);
  setFloat(args, "bb", &bms.balance_temperature);

  if (get(args, "armed", &v)) {
    const bool armed = v != 0;
    if (armed && !state.armed) state.armedAtMillis = eppgVirtualMillis;
    state.armed = armed;
  }
  if (get(args, "cruise", &v)) state.cruising = state.armed && v != 0;

  const float altitudeToShow = state.armed ? state.altitude : 0.0f;
  state.estimator.update(eppgVirtualMillis, state.altitude, state.battery.power,
                         state.armed);
  // Same feed as updateThermalHeadroom() in main.cpp.
  const bool escOn = esc.escState == TelemetryState::CONNECTED;
  const bool bmsOn = bms.bmsState == TelemetryState::CONNECTED;
  float temps[THERMAL_PART_COUNT] = {
    escOn ? esc.motor_temp : NAN, escOn ? esc.mos_temp : NAN,
    escOn ? esc.cap_temp : NAN, escOn && esc.mcu_temp != 0.0f ? esc.mcu_temp : NAN,
    bmsOn ? bms.t1_temperature : NAN, bmsOn ? bms.mos_temperature : NAN,
    bmsOn ? bms.balance_temperature : NAN};
  state.thermal.update(eppgVirtualMillis, state.battery.power, temps,
                       bmsOn ? bms.soc : NAN);

  if (main_screen == NULL) setupScreen();
  updateLvglMainScreen(state.deviceData, state.esc, state.bms, state.battery,
                       altitudeToShow, state.armed, state.cruising,
                       state.armedAtMillis);
  const ClimbEfficiencyResult& shown =
      state.mockActive ? state.mock : state.estimator.result();
  const ThermalHeadroomResult& heat = state.thermal.result();
  const float warn = state.mockActive ? state.mockWarnKw
                                      : (heat.valid ? heat.warnPowerKw : NAN);
  const float crit = state.mockActive ? state.mockCritKw
                                      : (heat.valid ? heat.critPowerKw : NAN);
  updateClimbEfficiencyDisplay(
      shown, climbBarZones(shown, warn, crit, state.estimator.config()),
      state.battery.power, state.armed);
}

const char* phaseName(ClimbEffPhase phase) {
  switch (phase) {
    case ClimbEffPhase::SETTLING: return "SETTLING";
    case ClimbEffPhase::UNSTEADY: return "UNSTEADY";
    case ClimbEffPhase::LOW_POWER: return "LOW_POWER";
    case ClimbEffPhase::NOT_CLIMBING: return "NOT_CLIMBING";
    case ClimbEffPhase::VALID: return "VALID";
    default: return "NO_DATA";
  }
}

// JSON has no NaN: invalid values go out as null.
void putVal(float value) {
  if (std::isnan(value) || std::isinf(value)) {
    printf("null");
  } else {
    printf("%.5g", value);
  }
}

void putNum(const char* key, float value) {
  printf("\"%s\":", key);
  putVal(value);
  putchar(',');
}

void cmdRender() {
  if (main_screen == NULL) setupScreen();
  lv_tick_inc(eppgVirtualMillis - state.lastRenderMillis);
  state.lastRenderMillis = eppgVirtualMillis;
  for (int i = 0; i < 4; i++) lv_timer_handler();
  lv_refr_now(main_display);

  const ClimbEfficiencyResult& r = state.estimator.result();
  printf("{\"t\":%lu,\"phase\":\"%s\",", eppgVirtualMillis, phaseName(r.phase));
  putNum("climb", r.climbRate);
  putNum("power", r.powerKw);
  putNum("cv", r.powerCv);
  putNum("yield", r.yield);
  putNum("rel", r.relativeYield);
  printf("\"curveValid\":%s,", r.curveValid ? "true" : "false");
  putNum("best", r.bestPowerKw);
  putNum("bestYield", r.bestYield);
  putNum("bandLo", r.bandLowKw);
  putNum("bandHi", r.bandHighKw);
  putNum("dataMin", r.dataMinKw);
  putNum("dataMax", r.dataMaxKw);
  putNum("learned", r.climbWeight);
  putNum("slope", r.slope);
  const ThermalHeadroomResult& heat = state.thermal.result();
  putNum("warn", heat.valid ? heat.warnPowerKw : NAN);
  putNum("crit", heat.valid ? heat.critPowerKw : NAN);
  printf("\"warnPart\":%d,", heat.warnLimiter);
  putNum("horizon", heat.horizonS);
  putNum("pack", heat.packKwh);
  printf("\"levelValid\":%s,", r.levelValid ? "true" : "false");
  putNum("level", r.levelPowerKw);
  putNum("levelMargin", r.levelMarginKw);
  putNum("levelW", r.levelWeight);
  printf("\"c\":[");
  putVal(r.c0);
  putchar(',');
  putVal(r.c1);
  putchar(',');
  putVal(r.c2);
  printf("],\"bins\":[");
  bool first = true;
  for (int i = 0; i < state.estimator.binCount(); ++i) {
    const ClimbEffBin& b = state.estimator.bin(i);
    if (b.weight < 0.05f) continue;
    printf("%s[%.4g,%.4g,%.4g]", first ? "" : ",", b.sumPowerKw / b.weight,
           b.sumClimb / b.weight, b.weight);
    first = false;
  }
  printf("],\"bar\":%s,\"frame\":%d}\n",
         getClimbEffBarEnabled() ? "true" : "false",
         SCREEN_WIDTH * SCREEN_HEIGHT * 2);
  fwrite(emulator_get_framebuffer(), 2, SCREEN_WIDTH * SCREEN_HEIGHT, stdout);
  fflush(stdout);
}

}  // namespace

int main() {
#ifdef _WIN32
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  state.deviceData.version_major = 8;
  state.deviceData.theme = 1;
  state.deviceData.metric_alt = true;
  state.deviceData.performance_mode = 1;
  state.deviceData.sea_pressure = 1013.25f;
  state.bms.mos_temperature = NAN;      // until a sample provides them
  state.bms.balance_temperature = NAN;

  static char line[4096];
  while (fgets(line, sizeof(line), stdin)) {
    char cmd[16] = {0};
    sscanf(line, "%15s", cmd);
    char* rest = line + strlen(cmd);
    Args args = parseArgs(rest);
    if (strcmp(cmd, "s") == 0) {
      cmdSample(args);
    } else if (strcmp(cmd, "render") == 0) {
      cmdRender();
    } else if (strcmp(cmd, "init") == 0) {
      cmdInit(args);
    } else if (strcmp(cmd, "mock") == 0) {
      double v;
      if (get(args, "off", &v) && v != 0) {
        state.mockActive = false;
      } else {
        ClimbEfficiencyResult& m = state.mock;
        m.levelValid = get(args, "level", &v) && v > 0;
        m.levelPowerKw = m.levelValid ? static_cast<float>(v) : NAN;
        m.levelMarginKw = get(args, "margin", &v) ? static_cast<float>(v) : NAN;
        m.curveValid = get(args, "lo", &v) && v > 0;
        m.bandLowKw = m.curveValid ? static_cast<float>(v) : NAN;
        m.bandHighKw = get(args, "hi", &v) ? static_cast<float>(v) : NAN;
        state.mockWarnKw = get(args, "warn", &v) ? static_cast<float>(v) : NAN;
        state.mockCritKw = get(args, "crit", &v) ? static_cast<float>(v) : NAN;
        state.mockActive = true;
      }
    } else if (strcmp(cmd, "bar") == 0) {
      double v;
      if (get(args, "v", &v)) setClimbEffBarEnabled(v != 0);
    } else if (strcmp(cmd, "cfg") == 0) {
      cmdConfig(args);
    } else if (strcmp(cmd, "reset") == 0) {
      state.estimator.reset();
      state.thermal.reset();
    } else if (strcmp(cmd, "quit") == 0) {
      break;
    }
  }
  return 0;
}
