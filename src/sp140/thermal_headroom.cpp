#include "sp140/thermal_headroom.h"

#include <math.h>

namespace {

// A step this long without data (disarmed on the ground, telemetry lost)
// means the previous temperature says nothing about the heating since.
constexpr float kMaxStepS = 60.0f;
// Shortest climb the horizon is allowed to shrink to.
constexpr float kMinHorizonS = 30.0f;

}  // namespace

ThermalHeadroom::ThermalHeadroom() { setConfig(ThermalHeadroomConfig()); }

ThermalHeadroom::ThermalHeadroom(const ThermalHeadroomConfig& config) {
  setConfig(config);
}

void ThermalHeadroom::setConfig(const ThermalHeadroomConfig& config) {
  config_ = config;
  if (config_.stepMs == 0) config_.stepMs = 10000;
  reset();
}

void ThermalHeadroom::reset() {
  started_ = false;
  stepStartMs_ = 0;
  sumP2_ = 0.0f;
  samples_ = 0;
  socStart_ = -1.0f;
  energyKwh_ = 0.0f;
  lastSoc_ = -1.0f;
  lastMs_ = 0;
  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    lastTemp_[i] = prevTemp_[i] = base_[i] = NAN;
    fitStarted_[i] = false;
    fitSeconds_[i] = 0.0f;
    heatStartC_[i] = NAN;
    heatP2S_[i] = 0.0f;
    gain_[i] = config_.parts[i].gainCPerKw2;
    p00_[i] = p01_[i] = p11_[i] = 0.0f;
    result_.baseC[i] = NAN;
    result_.gainC[i] = gain_[i];
  }
  result_.valid = false;
  result_.learnedMask = 0;
  result_.warnPowerKw = result_.critPowerKw = config_.maxPowerKw;
  result_.warnLimiter = result_.critLimiter = -1;
  result_.horizonS = config_.horizonMaxS;
  result_.packKwh = config_.packKwhDefault;
  result_.packLearned = false;
  result_.batteryGain = partGain(THERMAL_BATTERY);
}

void ThermalHeadroom::update(uint32_t nowMs, float powerKw,
                             const float tempsC[THERMAL_PART_COUNT],
                             float socPct) {
  if (isnan(powerKw) || powerKw < 0.0f) powerKw = 0.0f;

  // Energy since the first SOC reading, for the pack-size estimate.
  if (lastMs_ != 0 && nowMs > lastMs_) {
    const float dt = (nowMs - lastMs_) / 1000.0f;
    if (dt < 2.0f && socStart_ >= 0.0f) energyKwh_ += powerKw * dt / 3600.0f;
  }
  lastMs_ = nowMs;
  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    if (!config_.parts[i].trackBase && !isnan(tempsC[i]) && isnan(heatStartC_[i])) {
      heatStartC_[i] = tempsC[i];
    }
  }
  if (!isnan(socPct) && socPct > 0.0f) {
    if (socStart_ < 0.0f) socStart_ = socPct;
    lastSoc_ = socPct;
  }
  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    if (!isnan(tempsC[i])) lastTemp_[i] = tempsC[i];
  }

  if (!started_) {
    started_ = true;
    stepStartMs_ = nowMs;
  }
  sumP2_ += powerKw * powerKw;
  samples_++;
  if (nowMs - stepStartMs_ >= config_.stepMs) {
    step((nowMs - stepStartMs_) / 1000.0f, sumP2_ / samples_);
    stepStartMs_ = nowMs;
    sumP2_ = 0.0f;
    samples_ = 0;
  }
}

void ThermalHeadroom::step(float dtS, float meanP2) {
  ThermalHeadroomResult& r = result_;

  // Pack energy from SOC drop vs. energy drawn, once the drop is large
  // enough to trust.
  if (socStart_ >= 0.0f && lastSoc_ >= 0.0f) {
    const float drop = socStart_ - lastSoc_;
    if (drop >= config_.minSocDropForPackPct) {
      r.packKwh = fminf(fmaxf(energyKwh_ / (drop / 100.0f), 0.5f), 20.0f);
      r.packLearned = true;
    }
  }
  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    if (!isnan(heatStartC_[i])) heatP2S_[i] += meanP2 * dtS;
  }
  r.batteryGain = partGain(THERMAL_BATTERY);
  // The motor/ESC fits only learn while the motor works: idling on the
  // ground has no prop airflow and says nothing about cooling in flight.
  const float minP = config_.minLearnPowerKw;
  const bool powered = meanP2 >= minP * minP;

  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    const ThermalPartModel& m = config_.parts[i];
    const float t = lastTemp_[i];
    if (isnan(t)) continue;
    if (m.trackBase) {
      if (!fitStarted_[i]) {
        startFit(i, t);
      } else if (powered && !isnan(prevTemp_[i]) && dtS <= kMaxStepS) {
        updateFit(i, prevTemp_[i], t, dtS, meanP2);
        fitSeconds_[i] += dtS;
      }
    }
    prevTemp_[i] = t;
    r.baseC[i] = m.trackBase ? base_[i] : NAN;
    r.gainC[i] = partGain(i);
  }

  r.learnedMask = 0;
  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    if (partLearned(i)) r.learnedMask |= static_cast<uint8_t>(1u << i);
  }
  const bool wasValid = r.valid;
  r.valid = r.learnedMask != 0;
  if (!r.valid) return;

  int8_t warnWho, critWho;
  float horizon;
  const float warn = limitAll(false, &warnWho, &horizon);
  const float crit = limitAll(true, &critWho, nullptr);
  const float g = wasValid ? fminf(1.0f, dtS / config_.limitSmoothingS) : 1.0f;
  r.warnPowerKw += (warn - r.warnPowerKw) * g;
  r.critPowerKw += (crit - r.critPowerKw) * g;
  if (r.critPowerKw < r.warnPowerKw) r.critPowerKw = r.warnPowerKw;
  r.warnLimiter = warnWho;
  r.critLimiter = critWho;
  r.horizonS = horizon;
}

void ThermalHeadroom::startFit(int i, float firstTempC) {
  const ThermalPartModel& m = config_.parts[i];
  // Prior: the fleet's offset above the part's first reading, the fleet k.
  base_[i] = firstTempC + m.baseOverStartC;
  gain_[i] = m.gainCPerKw2;
  const float gainSd = config_.gainPriorRelSd * m.gainCPerKw2;
  p00_[i] = config_.basePriorSdC * config_.basePriorSdC;
  p01_[i] = 0.0f;
  p11_[i] = gainSd * gainSd;
  fitStarted_[i] = true;
}

// One recursive-least-squares step on the exact first-order discretisation
//   T = a T_prev + (1 - a) (T_base + k P^2),   a = exp(-dt / tau)
// i.e. y = T - a T_prev = phi . (T_base, k) with phi = (1 - a) (1, P^2).
// Holding power keeps only T_base + k P^2 identifiable; power changes
// (climbs vs. cruise) separate how hot the part runs from how steep it is.
void ThermalHeadroom::updateFit(int i, float prevC, float tempC, float dtS,
                                float meanP2) {
  const ThermalPartModel& m = config_.parts[i];
  const float g = 1.0f - expf(-dtS / m.tauS);
  const float phi0 = g;
  const float phi1 = g * meanP2;
  const float y = tempC - (1.0f - g) * prevC;
  const float lambda = config_.rlsForgetting;
  const float noise = config_.tempNoiseC * config_.tempNoiseC;

  const float pp0 = p00_[i] * phi0 + p01_[i] * phi1;
  const float pp1 = p01_[i] * phi0 + p11_[i] * phi1;
  const float s = lambda * noise + phi0 * pp0 + phi1 * pp1;
  const float k0 = pp0 / s;
  const float k1 = pp1 / s;
  const float err = y - (phi0 * base_[i] + phi1 * gain_[i]);
  base_[i] += k0 * err;
  gain_[i] += k1 * err;
  p00_[i] = (p00_[i] - k0 * pp0) / lambda;
  p01_[i] = (p01_[i] - k0 * pp1) / lambda;
  p11_[i] = (p11_[i] - k1 * pp1) / lambda;

  // Forgetting inflates directions the data does not excite (long steady
  // cruise); never let the uncertainty grow past the prior.
  const float gainSd = config_.gainPriorRelSd * m.gainCPerKw2;
  p00_[i] = fminf(p00_[i], config_.basePriorSdC * config_.basePriorSdC);
  p11_[i] = fminf(p11_[i], gainSd * gainSd);
  const float pmax = sqrtf(fmaxf(p00_[i] * p11_[i], 0.0f));
  p01_[i] = fmaxf(-pmax, fminf(p01_[i], pmax));

  gain_[i] = fmaxf(config_.gainMinRel * m.gainCPerKw2,
                   fminf(gain_[i], config_.gainMaxRel * m.gainCPerKw2));
  base_[i] = fmaxf(-30.0f, fminf(base_[i], 110.0f));
}

bool ThermalHeadroom::partLearned(int part) const {
  if (isnan(lastTemp_[part])) return false;
  const ThermalPartModel& m = config_.parts[part];
  if (m.trackBase) {
    const float gainSd = config_.gainPriorRelSd * m.gainCPerKw2;
    return fitStarted_[part] && fitSeconds_[part] >= config_.minLearnS &&
           p11_[part] <= config_.learnedGainVarFrac * gainSd * gainSd;
  }
  // Pack parts: the pack type is known and they have carried real load.
  return result_.packLearned && heatP2S_[part] >= config_.heatOnlyLearnKw2S;
}

float ThermalHeadroom::horizonFor(float powerKw) const {
  if (lastSoc_ < 0.0f || powerKw <= 0.1f) return config_.horizonMaxS;
  const float usableKwh =
      (lastSoc_ - config_.socReservePct) / 100.0f * result_.packKwh;
  if (usableKwh <= 0.0f) return kMinHorizonS;
  const float h = usableKwh * 3600.0f / powerKw;
  return fminf(fmaxf(h, kMinHorizonS), config_.horizonMaxS);
}

float ThermalHeadroom::partGain(int part) const {
  const ThermalPartModel& m = config_.parts[part];
  if (m.trackBase) return gain_[part];
  // Heating-only (pack) part: the curve for this pack type...
  const float pack = fmaxf(result_.packKwh, 0.5f);
  const float prior =
      m.gainCPerKw2 * powf(config_.packRefKwh / pack, m.packExponent);
  // ...blended toward this flight's own heating as evidence builds up. The
  // observed rate is floored at half the prior: a pack cooling off after the
  // charger would otherwise read as one that never heats.
  const float t = lastTemp_[part];
  const float e = heatP2S_[part];
  if (isnan(heatStartC_[part]) || isnan(t) || e <= 0.0f) return prior;
  const float observed = fmaxf((t - heatStartC_[part]) / e * m.tauS, 0.5f * prior);
  const float w = e / (e + config_.heatOnlyPriorKw2S);
  return (1.0f - w) * prior + w * observed;
}

float ThermalHeadroom::partLimit(int part, float thresholdC,
                                 float horizonS) const {
  const ThermalPartModel& m = config_.parts[part];
  const float t = lastTemp_[part];
  const float gain = partGain(part);
  if (isnan(t) || !(gain > 0.0f)) return config_.maxPowerKw;
  if (t >= thresholdC) return 0.0f;
  const float e = expf(-horizonS / m.tauS);
  const float base = (m.trackBase && !isnan(base_[part])) ? base_[part] : t;
  // Highest steady-state temperature that still ends below the threshold.
  const float tssMax = (thresholdC - t * e) / (1.0f - e);
  const float p2 = (tssMax - base) / gain;
  if (p2 <= 0.0f) return 0.0f;
  return fminf(sqrtf(p2), config_.maxPowerKw);
}

float ThermalHeadroom::limitAll(bool critical, int8_t* limiter,
                                float* horizonOut) const {
  float best = config_.maxPowerKw;
  int8_t who = -1;
  float bestHorizon = config_.horizonMaxS;
  for (int i = 0; i < THERMAL_PART_COUNT; ++i) {
    if (!(result_.learnedMask & (1u << i))) continue;
    const ThermalPartModel& m = config_.parts[i];
    const float th = (critical ? m.critC : m.warnC) - config_.thresholdMarginC;
    // The horizon depends on the power (a bigger climb empties the pack
    // sooner). Start from the longest horizon - the lowest limit - and
    // iterate up to the consistent one; it converges in a few steps.
    float h = config_.horizonMaxS;
    float p = partLimit(i, th, h);
    for (int it = 0; it < 5; ++it) {
      h = horizonFor(fmaxf(p, 0.5f));
      p = partLimit(i, th, h);
    }
    if (p < best) {
      best = p;
      who = static_cast<int8_t>(i);
      bestHorizon = h;
    }
  }
  if (limiter) *limiter = who;
  if (horizonOut) *horizonOut = bestHorizon;
  return best;
}
