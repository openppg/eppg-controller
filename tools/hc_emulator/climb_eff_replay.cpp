// Batch replay of the firmware climb efficiency estimator (no UI).
//
// Reads "t_ms altitude_m power_kw armed [soc motor esc_mos esc_cap esc_mcu
// battery bms_mos bms_balance]" lines on stdin (temperatures in C, "nan" when missing), the way
// the UI task would feed them, and prints the climb-efficiency estimator and
// thermal-headroom state as CSV every `every` ms.
// Config overrides are key=value arguments using the hc_emulator `cfg` keys:
//
//   climb_eff_replay every=1000 window=10000 q=0.028 < flight.txt
//
// Used by tools/hc_emulator/analysis/ to score the estimator against the
// ground truth derived from real flight logs.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "sp140/climb_efficiency.h"
#include "sp140/thermal_headroom.h"

namespace {

const char* kPartKeys[THERMAL_PART_COUNT] = {"motor", "mos", "cap", "mcu", "batt",
                                             "bmsmos", "bmsbal"};

// Thermal overrides: k_<part>=, tau_<part>=, plus horizon= and packkwh=.
bool parseThermalArg(const std::string& key, double v, ThermalHeadroomConfig* th) {
  if (key == "horizon") {
    th->horizonMaxS = static_cast<float>(v);
    return true;
  }
  if (key == "packkwh") {
    th->packKwhDefault = static_cast<float>(v);
    return true;
  }
  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    if (key == std::string("k_") + kPartKeys[i]) {
      th->parts[i].gainCPerKw2 = static_cast<float>(v);
      return true;
    }
    if (key == std::string("tau_") + kPartKeys[i]) {
      th->parts[i].tauS = static_cast<float>(v);
      return true;
    }
  }
  return false;
}

bool parseArg(const char* arg, ClimbEfficiencyConfig* cfg, uint32_t* every,
              ThermalHeadroomConfig* th) {
  const char* eq = strchr(arg, '=');
  if (!eq) return false;
  const std::string key(arg, eq - arg);
  const double v = strtod(eq + 1, NULL);
  if (parseThermalArg(key, v, th)) return true;
  if (key == "every") *every = static_cast<uint32_t>(v);
  else if (key == "window") cfg->windowMs = static_cast<uint32_t>(v);
  else if (key == "settle") cfg->settleMs = static_cast<uint32_t>(v);
  else if (key == "learn") cfg->learnEveryMs = static_cast<uint32_t>(v);
  else if (key == "cv") cfg->maxPowerCv = static_cast<float>(v);
  else if (key == "minp") cfg->minPowerKw = static_cast<float>(v);
  else if (key == "minclimb") cfg->minClimbMs = static_cast<float>(v);
  else if (key == "minalt") cfg->minAltitudeM = static_cast<float>(v);
  else if (key == "binhalflife") cfg->binHalfLife = static_cast<float>(v);
  else if (key == "q") cfg->curvatureQ = static_cast<float>(v);
  else if (key == "maxp") cfg->maxPowerKw = static_cast<float>(v);
  else if (key == "climbev") cfg->climbEvidenceMs = static_cast<float>(v);
  else if (key == "minclimbw") cfg->minClimbWeight = static_cast<float>(v);
  else if (key == "band") cfg->bandFraction = static_cast<float>(v);
  else if (key == "binw") cfg->binWidthKw = static_cast<float>(v);
  else if (key == "minbinw") cfg->minBinWeight = static_cast<float>(v);
  else if (key == "levelband") cfg->levelBandMs = static_cast<float>(v);
  else if (key == "priorslope") cfg->priorSlope = static_cast<float>(v);
  else if (key == "levelhalflife") cfg->levelHalfLife = static_cast<float>(v);
  else if (key == "minlevelw") cfg->minLevelWeight = static_cast<float>(v);
  else if (key == "minlevelp") cfg->minLevelPowerKw = static_cast<float>(v);
  else if (key == "margink") cfg->levelMarginK = static_cast<float>(v);
  else if (key == "marginfloor") cfg->levelMarginFloorKw = static_cast<float>(v);
  else if (key == "priorspread") cfg->levelPriorSpreadKw = static_cast<float>(v);
  else if (key == "priorpts") cfg->levelPriorPoints = static_cast<float>(v);
  else if (key == "maxmargin") cfg->maxShownMarginKw = static_cast<float>(v);
  else return false;
  return true;
}

void put(float v) {
  if (std::isnan(v) || std::isinf(v)) {
    fputs(",", stdout);
  } else {
    printf(",%.4g", v);
  }
}

}  // namespace

int main(int argc, char** argv) {
  ClimbEfficiencyConfig cfg;
  ThermalHeadroomConfig thCfg;
  uint32_t every = 1000;
  for (int i = 1; i < argc; ++i) {
    if (!parseArg(argv[i], &cfg, &every, &thCfg)) {
      fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 2;
    }
  }
  ClimbEfficiencyEstimator est(cfg);
  ThermalHeadroom thermal(thCfg);

  puts("t_ms,phase,climb,power,yield,curve_valid,best,band_lo,band_hi,"
       "level_valid,level,level_w,climb_w,slope,level_margin,"
       "th_valid,warn_kw,crit_kw,warn_part,crit_part,horizon_s,pack_kwh,"
       "base_motor,base_mos,base_cap,base_mcu,"
       "gain_motor,gain_mos,gain_cap,gain_mcu,gain_batt,gain_bmsmos,gain_bmsbal");
  char line[256];
  bool first = true;
  uint32_t nextOut = 0;
  while (fgets(line, sizeof(line), stdin)) {
    unsigned long t;
    float alt, power;
    int armed;
    float soc = NAN;
    float temps[THERMAL_PART_COUNT];
    for (float& v : temps) v = NAN;
    const int n = sscanf(line, "%lu %f %f %d %f %f %f %f %f %f %f %f", &t, &alt,
                         &power, &armed, &soc, &temps[0], &temps[1], &temps[2],
                         &temps[3], &temps[4], &temps[5], &temps[6]);
    if (n < 4) continue;
    est.update(static_cast<uint32_t>(t), alt, power, armed != 0);
    thermal.update(static_cast<uint32_t>(t), power, temps, soc);
    if (first) {
      nextOut = static_cast<uint32_t>(t);
      first = false;
    }
    if (static_cast<uint32_t>(t) < nextOut) continue;
    nextOut = static_cast<uint32_t>(t) + every;

    const ClimbEfficiencyResult& r = est.result();
    printf("%lu,%d", t, static_cast<int>(r.phase));
    put(r.climbRate);
    put(r.powerKw);
    put(r.yield);
    printf(",%d", r.curveValid ? 1 : 0);
    put(r.bestPowerKw);
    put(r.bandLowKw);
    put(r.bandHighKw);
    printf(",%d", r.levelValid ? 1 : 0);
    put(r.levelPowerKw);
    put(r.levelWeight);
    put(r.climbWeight);
    put(r.slope);
    put(r.levelMarginKw);
    const ThermalHeadroomResult& h = thermal.result();
    printf(",%d", h.valid ? 1 : 0);
    put(h.warnPowerKw);
    put(h.critPowerKw);
    printf(",%d,%d", h.warnLimiter, h.critLimiter);
    put(h.horizonS);
    put(h.packKwh);
    for (int i = 0; i < THERMAL_BATTERY; ++i) put(h.baseC[i]);
    for (int i = 0; i < THERMAL_PART_COUNT; ++i) put(h.gainC[i]);
    putchar('\n');
  }
  return 0;
}
