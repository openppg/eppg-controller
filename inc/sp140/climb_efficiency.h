#ifndef INC_SP140_CLIMB_EFFICIENCY_H_
#define INC_SP140_CLIMB_EFFICIENCY_H_

// Climb efficiency (GitHub issue #57): learns, in flight, from the hand
// controller's barometer and pack power only
//
//   - the power that holds level flight (L), and
//   - the power band that climbs most efficiently (most altitude per Wh).
//
// Pure C++ (no Arduino/FreeRTOS) so the same code runs on the controller, in
// the native unit tests and in the desktop emulator / fleet replay tools.
//
// Model. Near trim a paramotor's steady climb rate w (m/s) at pack power P
// (kW) is
//
//     w(P) = s (P - L) - q s (P - L)^2
//
// L = level-flight power, s = climb gained per extra kW near level, and q =
// how quickly extra thrust stops paying (prop/motor losses). Climb per Wh,
// w/P, then peaks at
//
//     P* = sqrt(L^2 + L / q)
//
// and s cancels out of where the peak and the 95 % band are, so the band
// needs only L and q. L is what varies between setups (2.9-6.6 kW across 30
// real SP140 flights on 8 wings) and is learned continuously from steady,
// near-level flight. q is set by the prop and airspeed, so it is a fixed
// default: in the fleet data climb is linear in power (q ~ 0) up to ~5 kW
// above level, the few steady climbs beyond that start to bend, and the
// actuator-disk + motor-loss physics gives 0.017; combined, q = 0.016. That
// puts the best climb at ~14 kW for a 3 kW-level setup and the 5 % band from
// ~10 kW to full power; heavier setups move it up. See
// tools/hc_emulator/analysis/ for the fleet analysis.
//
// Measurements come from steady windows only: altitude slope (least squares)
// over windowMs after power has held steady for windowMs + settleMs, so the
// pendulum/pitch transient after a throttle change never counts.
//
// Displayed yield (for the emulator/app): m/Wh = 3.6 * w / P.

#include <stdint.h>

struct ClimbEfficiencyConfig {
  uint32_t sampleMs = 100;        // inputs are averaged into slots this long
  uint32_t windowMs = 10000;      // regression window for climb rate
  uint32_t settleMs = 3000;       // extra steady-power time required first
  float minPowerKw = 1.5f;        // below this it is not powered flight
  float minClimbMs = 0.3f;        // below this it is cruise/descent, not a climb
  float maxPowerCv = 0.12f;       // max stddev/mean of power while "steady"
  float minPowerStdKw = 0.10f;    // stddev always tolerated (near-zero power)
  float minAltitudeM = 15.0f;     // ignore takeoff run / ground (AGL)
  uint32_t learnEveryMs = 1000;   // how often a steady window is learned from

  // Level-flight power: steady windows within +/-levelBandMs of level each
  // contribute their power corrected to w = 0 along the slope s, with
  // recency weighting (half-life in level points, ~1 per second).
  // Tuned by replaying 30 real flights through this estimator against their
  // two-baro + GPS ground truth: mean error 0.15 kW (worst 0.31) on the 18
  // with plenty of level flight; the tick shows after a median 3.2 min.
  float levelBandMs = 0.5f;
  float priorSlope = 0.32f;       // s until climbs measure it (fleet median)
  float levelHalfLife = 300.0f;   // ~5 min of level flight
  float minLevelWeight = 8.0f;    // level points before the tick can show
  // No SP140 holds level on less (fleet minimum 2.9 kW). Soaring at low power
  // in rising air otherwise reads as an impossibly low level power.
  float minLevelPowerKw = 2.0f;

  // Certainty margin around the level power (the brackets on the tick):
  //
  //   margin = sqrt(k^2 * var / nEff + floor^2)
  //
  //   var  = recency-weighted spread of the level samples, shrunk toward
  //          levelPriorSpreadKw while there are only a few of them
  //   nEff = effective number of samples, (sum w)^2 / sum(w^2)
  //   k    = z * sqrt(c): z sets the coverage, c = learn points per
  //          independent sample (1 s points from 13 s windows overlap)
  //   floor = what more data cannot remove (slope correction, air mass)
  //
  // Calibrated by replaying 18 real flights: the true level power is inside
  // the margin 91 % of the time the tick is shown; the margin shrinks from
  // <= 1.5 kW when it appears to ~0.45 kW after 5 min and ~0.32 kW (the
  // floor dominates) later in the flight.
  float levelMarginK = 10.0f;
  float levelMarginFloorKw = 0.25f;
  float levelPriorSpreadKw = 0.6f;
  float levelPriorPoints = 5.0f;
  float maxShownMarginKw = 1.5f;  // the tick appears once the margin is below

  // Best-climb band.
  float curvatureQ = 0.016f;      // q, per kW (fleet data + prop physics)
  float maxPowerKw = 16.0f;       // SP140 peak; the band never goes past it
  float bandFraction = 0.95f;     // band = climb per Wh within 5 % of best
  float climbEvidenceMs = 0.5f;   // a "real" steady climb for the gate below
  float minClimbWeight = 10.0f;   // climb points before the band shows

  // Power bins behind the slope estimate (and the emulator's plots).
  float binWidthKw = 1.0f;
  float binHalfLife = 60.0f;      // recency within one power bin, in points
  float minBinWeight = 4.0f;      // decayed weight a bin needs to count
};

enum class ClimbEffPhase : uint8_t {
  NO_DATA = 0,   // disarmed, below minAltitudeM, or after a data gap
  SETTLING,      // steady, but the window is not full yet
  UNSTEADY,      // power is changing
  LOW_POWER,     // gliding / idle
  NOT_CLIMBING,  // powered and steady but level or descending (cruise)
  VALID,         // steady powered climb: yield is valid
};

struct ClimbEfficiencyResult {
  ClimbEffPhase phase;
  float climbRate;      // m/s, regression slope over the window (NaN if none)
  float powerKw;        // mean power over the window (NaN if none)
  float powerCv;        // power spread over window + settle
  float yield;          // w / P in m/s per kW; NaN unless phase == VALID

  bool levelValid;      // enough near-level flight to trust levelPowerKw
  float levelPowerKw;   // L: power that holds altitude (NaN until learned)
  float levelMarginKw;  // +/- certainty on L (NaN until learned)
  float levelWeight;    // effective number of near-level points behind it

  bool curveValid;      // level learned and the pilot has climbed
  float bestPowerKw;    // P*: most altitude per Wh
  float bandLowKw;      // band where climb per Wh >= bandFraction * best
  float bandHighKw;
  float slope;          // s, m/s per kW (NaN until climbs measure it)
  float bestYield;      // s * shape at P* (NaN without s)
  float relativeYield;  // yield / bestYield (NaN unless both valid)
  float climbWeight;    // (decayed) steady-climb points seen
  float dataMinKw;      // powered range the bins cover
  float dataMaxKw;
  float c0, c1, c2;     // the model expanded: w = c0 + c1 P + c2 P^2
};

struct ClimbEffBin {
  float weight;         // decayed number of learned points
  float sumPowerKw;     // weighted sums (divide by weight for the mean)
  float sumClimb;
};

class ClimbEfficiencyEstimator {
 public:
  static constexpr int kMaxSamples = 160;
  static constexpr int kMaxBins = 32;

  ClimbEfficiencyEstimator();
  explicit ClimbEfficiencyEstimator(const ClimbEfficiencyConfig& config);

  // Replaces the configuration and forgets everything.
  void setConfig(const ClimbEfficiencyConfig& config);
  const ClimbEfficiencyConfig& config() const { return config_; }

  // Forget the window and everything learned.
  void reset();

  // Feed one reading (call at the UI rate, ~30 Hz). altitudeM is AGL
  // relative to the arming point; pass NaN for an invalid reading.
  void update(uint32_t nowMs, float altitudeM, float powerKw, bool armed);

  const ClimbEfficiencyResult& result() const { return result_; }
  const ClimbEffBin& bin(int index) const { return bins_[index]; }
  int binCount() const { return kMaxBins; }

  // Model climb rate at a power (NaN until level power and slope are known).
  float predictedClimb(float powerKw) const;

 private:
  struct Sample {
    uint32_t t;
    float altitude;
    float power;
  };

  void resetWindow();
  void pushSample(uint32_t t, float altitude, float power);
  void evaluate(uint32_t nowMs, bool armed);
  void learnBins(float powerKw, float climb);
  void learnLevel(float powerKw, float climb);
  void updateModel();

  ClimbEfficiencyConfig config_;
  ClimbEfficiencyResult result_;

  Sample samples_[kMaxSamples];
  int head_ = 0;      // index of the oldest sample
  int count_ = 0;

  // Slot accumulator (averages readings into sampleMs slots)
  uint32_t slotStartMs_ = 0;
  uint32_t slotSumDt_ = 0;
  float slotSumAlt_ = 0.0f;
  float slotSumPower_ = 0.0f;
  uint16_t slotCount_ = 0;

  bool haveLastUpdate_ = false;
  uint32_t lastUpdateMs_ = 0;
  bool haveLearned_ = false;
  uint32_t lastLearnMs_ = 0;

  ClimbEffBin bins_[kMaxBins];
  // Recency-weighted mean and spread of level samples (West's update):
  // weight sum, sum of squared weights, mean, sum of w * (x - mean)^2.
  float levelW_ = 0.0f;
  float levelW2_ = 0.0f;
  float levelMean_ = 0.0f;
  float levelM2_ = 0.0f;
  float climbWeight_ = 0.0f;
};

// What the power bar shows right of the level tick. Green: powers within
// bandFraction of the best climb per Wh that can be *sustained* - the band
// ends at the efficiency peak or at the thermal warning limit, whichever is
// lower (below the peak, more power is always more efficient, so when heat
// caps the power the best sustainable climb is right at the cap). Yellow:
// holding the power would reach a warning temperature; red: a critical one.
struct ClimbBarZones {
  bool green, yellow, red;
  float greenLowKw, greenHighKw;
  float yellowLowKw, yellowHighKw;
  float redLowKw, redHighKw;
};

// warnKw / critKw: thermal limits (NaN or >= maxPowerKw: none).
ClimbBarZones climbBarZones(const ClimbEfficiencyResult& r, float warnKw,
                            float critKw, const ClimbEfficiencyConfig& config);

// Unit conversions for display (yield is m/s per kW == m per kJ).
inline float climbYieldToMetersPerWh(float yield) { return yield * 3.6f; }
inline float climbYieldToFeetPerWh(float yield) {
  return yield * 3.6f * 3.28084f;
}

#endif  // INC_SP140_CLIMB_EFFICIENCY_H_
