#include "sp140/climb_efficiency.h"

#include <math.h>
#include <string.h>

namespace {

// A reading gap longer than this (UI stall, baro dropout) restarts the window:
// a regression across the gap would mix two unrelated flight segments.
constexpr uint32_t kGapResetMs = 1000;

// Bins closer than this to level power carry almost no information about the
// slope (the model is ~0 there), so they are left out of its fit.
constexpr float kSlopeMinLeverKw = 1.0f;

inline float maxf(float a, float b) { return a > b ? a : b; }
inline float minf(float a, float b) { return a < b ? a : b; }

// Climb per Wh relative to s: w / (s P) for the model w = s x (1 - q x),
// x = P - L. The band and best power depend only on this shape.
inline float yieldShape(float p, float level, float q) {
  const float x = p - level;
  return x * (1.0f - q * x) / p;
}

}  // namespace

ClimbEfficiencyEstimator::ClimbEfficiencyEstimator() {
  setConfig(ClimbEfficiencyConfig());
}

ClimbEfficiencyEstimator::ClimbEfficiencyEstimator(
    const ClimbEfficiencyConfig& config) {
  setConfig(config);
}

void ClimbEfficiencyEstimator::setConfig(const ClimbEfficiencyConfig& config) {
  config_ = config;
  if (config_.sampleMs == 0) config_.sampleMs = 100;
  if (config_.binWidthKw <= 0.0f) config_.binWidthKw = 1.0f;
  // Coarsen the slots rather than overflow the ring buffer if a long window
  // is requested.
  const uint32_t span = config_.windowMs + config_.settleMs;
  if (span / config_.sampleMs + 2 > static_cast<uint32_t>(kMaxSamples)) {
    config_.sampleMs = span / (kMaxSamples - 2) + 1;
  }
  reset();
}

void ClimbEfficiencyEstimator::reset() {
  memset(bins_, 0, sizeof(bins_));
  haveLearned_ = false;
  lastLearnMs_ = 0;
  levelW_ = levelW2_ = levelMean_ = levelM2_ = 0.0f;
  climbWeight_ = 0.0f;

  ClimbEfficiencyResult& r = result_;
  r.levelValid = false;
  r.levelPowerKw = NAN;
  r.levelMarginKw = NAN;
  r.levelWeight = 0.0f;
  r.curveValid = false;
  r.bestPowerKw = r.bandLowKw = r.bandHighKw = NAN;
  r.slope = r.bestYield = NAN;
  r.climbWeight = 0.0f;
  r.dataMinKw = r.dataMaxKw = NAN;
  r.c0 = r.c1 = r.c2 = NAN;
  resetWindow();
}

void ClimbEfficiencyEstimator::resetWindow() {
  head_ = 0;
  count_ = 0;
  slotCount_ = 0;
  slotSumDt_ = 0;
  slotSumAlt_ = 0.0f;
  slotSumPower_ = 0.0f;
  haveLastUpdate_ = false;

  result_.phase = ClimbEffPhase::NO_DATA;
  result_.climbRate = NAN;
  result_.powerKw = NAN;
  result_.powerCv = NAN;
  result_.yield = NAN;
  result_.relativeYield = NAN;
}

void ClimbEfficiencyEstimator::update(uint32_t nowMs, float altitudeM,
                                      float powerKw, bool armed) {
  if (!armed || isnan(altitudeM) || isnan(powerKw)) {
    resetWindow();
    return;
  }
  if (haveLastUpdate_ && (nowMs - lastUpdateMs_) > kGapResetMs) {
    resetWindow();
  }
  haveLastUpdate_ = true;
  lastUpdateMs_ = nowMs;

  // Close the current slot once it spans sampleMs, then start a new one with
  // this reading. Averaging within a slot is a cheap boxcar pre-filter.
  if (slotCount_ > 0 && (nowMs - slotStartMs_) >= config_.sampleMs) {
    const float n = static_cast<float>(slotCount_);
    pushSample(slotStartMs_ + slotSumDt_ / slotCount_, slotSumAlt_ / n,
               slotSumPower_ / n);
    slotCount_ = 0;
    evaluate(nowMs, armed);
  }
  if (slotCount_ == 0) {
    slotStartMs_ = nowMs;
    slotSumDt_ = 0;
    slotSumAlt_ = 0.0f;
    slotSumPower_ = 0.0f;
  }
  slotSumDt_ += nowMs - slotStartMs_;
  slotSumAlt_ += altitudeM;
  slotSumPower_ += powerKw < 0.0f ? 0.0f : powerKw;  // regen/noise below 0
  slotCount_++;
}

void ClimbEfficiencyEstimator::pushSample(uint32_t t, float altitude,
                                          float power) {
  const int tail = (head_ + count_) % kMaxSamples;
  samples_[tail] = {t, altitude, power};
  if (count_ < kMaxSamples) {
    count_++;
  } else {
    head_ = (head_ + 1) % kMaxSamples;
  }
}

void ClimbEfficiencyEstimator::evaluate(uint32_t nowMs, bool armed) {
  ClimbEfficiencyResult& r = result_;
  if (count_ == 0) return;

  const Sample& newest = samples_[(head_ + count_ - 1) % kMaxSamples];
  const uint32_t extSpan = config_.windowMs + config_.settleMs;

  // Pass 1: means. Power over window + settle; time/altitude/power over the
  // window. Times are seconds relative to the newest sample and altitudes are
  // relative to the newest altitude, which keeps float sums well conditioned.
  int nExt = 0, nWin = 0;
  uint32_t oldestExtAge = 0, oldestWinAge = 0;
  float sumPExt = 0.0f, sumPWin = 0.0f, sumT = 0.0f, sumA = 0.0f;
  float minAlt = newest.altitude;
  for (int i = count_ - 1; i >= 0; --i) {
    const Sample& s = samples_[(head_ + i) % kMaxSamples];
    const uint32_t age = newest.t - s.t;
    if (age > extSpan) break;
    nExt++;
    sumPExt += s.power;
    oldestExtAge = age;
    if (age <= config_.windowMs) {
      nWin++;
      sumPWin += s.power;
      sumT += -static_cast<float>(age) / 1000.0f;
      sumA += s.altitude - newest.altitude;
      minAlt = minf(minAlt, s.altitude);
      oldestWinAge = age;
    }
  }
  const float meanPExt = sumPExt / nExt;
  const float meanPWin = sumPWin / nWin;
  const float meanT = sumT / nWin;
  const float meanA = sumA / nWin;

  // Pass 2: centred sums (power spread, regression slope).
  float varPExt = 0.0f, sTT = 0.0f, sTA = 0.0f;
  for (int i = count_ - 1, k = 0; i >= 0 && k < nExt; --i, ++k) {
    const Sample& s = samples_[(head_ + i) % kMaxSamples];
    const uint32_t age = newest.t - s.t;
    const float dp = s.power - meanPExt;
    varPExt += dp * dp;
    if (age <= config_.windowMs) {
      const float dt = -static_cast<float>(age) / 1000.0f - meanT;
      const float da = (s.altitude - newest.altitude) - meanA;
      sTT += dt * dt;
      sTA += dt * da;
    }
  }
  const float stdPExt = sqrtf(varPExt / nExt);

  r.powerKw = meanPWin;
  r.powerCv = meanPExt > 0.05f ? stdPExt / meanPExt : NAN;
  r.climbRate = (nWin >= 3 && oldestWinAge >= 1000 && sTT > 0.0f)
      ? sTA / sTT : NAN;

  const bool steady =
      stdPExt <= maxf(config_.maxPowerCv * meanPExt, config_.minPowerStdKw);
  // Slots can run a little longer than sampleMs, so allow two slots of slack.
  const bool fullSpan = oldestExtAge + 2 * config_.sampleMs >= extSpan;
  const bool aboveGround = minAlt >= config_.minAltitudeM;

  if (!aboveGround || isnan(r.climbRate)) {
    r.phase = ClimbEffPhase::NO_DATA;
  } else if (meanPWin < config_.minPowerKw) {
    r.phase = ClimbEffPhase::LOW_POWER;
  } else if (!steady) {
    r.phase = ClimbEffPhase::UNSTEADY;
  } else if (!fullSpan) {
    r.phase = ClimbEffPhase::SETTLING;
  } else if (r.climbRate < config_.minClimbMs) {
    r.phase = ClimbEffPhase::NOT_CLIMBING;
  } else {
    r.phase = ClimbEffPhase::VALID;
  }
  r.yield = (r.phase == ClimbEffPhase::VALID) ? r.climbRate / r.powerKw : NAN;

  // Every steady, settled window above the ground is learned from, at most
  // once per learnEveryMs.
  if (armed && aboveGround && steady && fullSpan && !isnan(r.climbRate) &&
      (!haveLearned_ || (nowMs - lastLearnMs_) >= config_.learnEveryMs)) {
    lastLearnMs_ = nowMs;
    haveLearned_ = true;
    learnBins(meanPWin, r.climbRate);
    learnLevel(meanPWin, r.climbRate);
    if (meanPWin >= config_.minPowerKw &&
        r.climbRate >= config_.climbEvidenceMs) {
      climbWeight_ += 1.0f;
    }
    updateModel();
  }

  r.relativeYield = (r.phase == ClimbEffPhase::VALID && r.bestYield > 0.0f)
      ? r.yield / r.bestYield : NAN;
}

void ClimbEfficiencyEstimator::learnBins(float powerKw, float climb) {
  // Bins are centred on multiples of binWidthKw, so a steady 4.0 kW lands in
  // one bin instead of straddling two.
  int index = static_cast<int>(
      floorf(maxf(0.0f, powerKw) / config_.binWidthKw + 0.5f));
  if (index >= kMaxBins) index = kMaxBins - 1;
  // Recency within a bin: new points at a power displace older ones at that
  // power, while bins that get nothing new (the takeoff climb, during an
  // hour of cruise) keep what they learned.
  ClimbEffBin& b = bins_[index];
  const float decay = powf(0.5f, 1.0f / maxf(1.0f, config_.binHalfLife));
  b.weight = b.weight * decay + 1.0f;
  b.sumPowerKw = b.sumPowerKw * decay + powerKw;
  b.sumClimb = b.sumClimb * decay + climb;
}

void ClimbEfficiencyEstimator::learnLevel(float powerKw, float climb) {
  ClimbEfficiencyResult& r = result_;
  if (powerKw < config_.minLevelPowerKw || fabsf(climb) > config_.levelBandMs) {
    return;
  }
  // Correct a slight climb or sink back to level along the slope: holding
  // level would have taken climb / s less power.
  const float slope = isnan(r.slope) ? config_.priorSlope : r.slope;
  const float levelEquivalent = powerKw - climb / slope;

  // Recency-weighted mean and spread, updated in place: 4 floats of state,
  // one sqrt per learn step.
  const float decay = powf(0.5f, 1.0f / maxf(1.0f, config_.levelHalfLife));
  levelW_ = levelW_ * decay + 1.0f;
  levelW2_ = levelW2_ * decay * decay + 1.0f;
  levelM2_ *= decay;
  const float delta = levelEquivalent - levelMean_;
  levelMean_ += delta / levelW_;
  levelM2_ += delta * (levelEquivalent - levelMean_);

  // Spread, shrunk toward the prior while samples are few, then the standard
  // error of the mean over the effective sample count.
  const float priorVar = config_.levelPriorSpreadKw * config_.levelPriorSpreadKw;
  const float var = (levelM2_ + config_.levelPriorPoints * priorVar) /
                    (levelW_ + config_.levelPriorPoints);
  const float nEff = levelW_ * levelW_ / levelW2_;
  const float k = config_.levelMarginK;
  const float floorKw = config_.levelMarginFloorKw;
  r.levelWeight = nEff;
  r.levelPowerKw = levelMean_;
  r.levelMarginKw = sqrtf(k * k * var / nEff + floorKw * floorKw);
  // Shown once certain enough; hidden again only if it becomes clearly
  // uncertain, so the tick does not flicker at the threshold.
  const float hideAbove = (r.levelValid ? 1.5f : 1.0f) * config_.maxShownMarginKw;
  r.levelValid = nEff >= config_.minLevelWeight && r.levelMarginKw <= hideAbove;
}

void ClimbEfficiencyEstimator::updateModel() {
  ClimbEfficiencyResult& r = result_;
  const float q = config_.curvatureQ;
  r.climbWeight = climbWeight_;

  // Powered range the bins cover (for the emulator's plots).
  float minP = 1e9f, maxP = -1e9f;
  for (int i = 0; i < kMaxBins; ++i) {
    const ClimbEffBin& b = bins_[i];
    if (b.weight < config_.minBinWeight) continue;
    const float p = b.sumPowerKw / b.weight;
    if (p < config_.minPowerKw) continue;
    minP = minf(minP, p);
    maxP = maxf(maxP, p);
  }
  r.dataMinKw = maxP >= minP ? minP : NAN;
  r.dataMaxKw = maxP >= minP ? maxP : NAN;

  if (!r.levelValid || !(q > 0.0f)) {
    r.curveValid = false;
    return;
  }
  const float level = r.levelPowerKw;

  // Slope s: one-parameter least squares through the bins with the shape
  // fixed, w = s * x (1 - q x). Only bins well away from level carry it.
  float num = 0.0f, den = 0.0f;
  for (int i = 0; i < kMaxBins; ++i) {
    const ClimbEffBin& b = bins_[i];
    if (b.weight < config_.minBinWeight) continue;
    const float p = b.sumPowerKw / b.weight;
    const float x = p - level;
    if (p < config_.minPowerKw || fabsf(x) < kSlopeMinLeverKw) continue;
    const float f = x * (1.0f - q * x);
    num += b.weight * f * (b.sumClimb / b.weight);
    den += b.weight * f * f;
  }
  const float s = den > 0.0f ? num / den : NAN;
  r.slope = (s > 0.05f && s < 2.0f) ? s : NAN;

  // The band only appears once the pilot has actually climbed this flight.
  r.curveValid = climbWeight_ >= config_.minClimbWeight;
  if (!r.curveValid) return;

  // Peak of the yield shape (or full power, if the peak is beyond what the
  // motor can do), then the band where the shape is >= bandFraction of it:
  // q x^2 - (1 - c) x + c L <= 0 with c = fraction * shape(best).
  const float best = minf(sqrtf(level * level + level / q), config_.maxPowerKw);
  const float c = config_.bandFraction * yieldShape(best, level, q);
  const float b = 1.0f - c;
  const float disc = maxf(0.0f, b * b - 4.0f * q * c * level);
  r.bestPowerKw = best;
  r.bandLowKw = level + (b - sqrtf(disc)) / (2.0f * q);
  r.bandHighKw = minf(level + (b + sqrtf(disc)) / (2.0f * q), config_.maxPowerKw);

  if (isnan(r.slope)) {
    r.bestYield = NAN;
    r.c0 = r.c1 = r.c2 = NAN;
  } else {
    r.bestYield = r.slope * yieldShape(best, level, q);
    // s x - q s x^2 with x = P - L, expanded in P.
    r.c0 = -r.slope * level - q * r.slope * level * level;
    r.c1 = r.slope + 2.0f * q * r.slope * level;
    r.c2 = -q * r.slope;
  }
}

float ClimbEfficiencyEstimator::predictedClimb(float powerKw) const {
  const ClimbEfficiencyResult& r = result_;
  if (!r.levelValid || isnan(r.slope)) return NAN;
  const float x = powerKw - r.levelPowerKw;
  return r.slope * x * (1.0f - config_.curvatureQ * x);
}

ClimbBarZones climbBarZones(const ClimbEfficiencyResult& r, float warnKw,
                            float critKw, const ClimbEfficiencyConfig& config) {
  ClimbBarZones z = {};
  const float maxP = config.maxPowerKw;
  const float warn = isnan(warnKw) ? maxP : minf(warnKw, maxP);
  const float crit = isnan(critKw) ? maxP : minf(maxf(critKw, warn), maxP);
  if (warn < maxP && crit > warn) {
    z.yellow = true;
    z.yellowLowKw = warn;
    z.yellowHighKw = crit;
  }
  if (crit < maxP) {
    z.red = true;
    z.redLowKw = crit;
    z.redHighKw = maxP;
  }

  const float q = config.curvatureQ;
  if (!r.curveValid || !r.levelValid || !(q > 0.0f)) return z;
  const float level = r.levelPowerKw;
  const float peak = minf(sqrtf(level * level + level / q), maxP);
  const float best = minf(peak, warn);
  if (best <= level) return z;  // cannot even hold level without heating
  // Band where the yield shape is >= bandFraction of its value at `best`.
  const float c = config.bandFraction * yieldShape(best, level, q);
  const float b = 1.0f - c;
  const float disc = maxf(0.0f, b * b - 4.0f * q * c * level);
  const float low = level + (b - sqrtf(disc)) / (2.0f * q);
  // Heat-limited: green stops at the limit. Otherwise it runs past the peak
  // to where the yield drops below the band again (or full power).
  const bool heatLimited = warn < peak;
  const float high = heatLimited
      ? best : minf(level + (b + sqrtf(disc)) / (2.0f * q), warn);
  if (high > low) {
    z.green = true;
    z.greenLowKw = low;
    z.greenHighKw = high;
  }
  return z;
}
