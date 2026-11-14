#include "platform/runtime.h"
#include "sp140/altimeter.h"
#include "sp140/structs.h"
#include "sp140/time_utils.h"

#include "platform/barometer.h"

platform::Barometer bmp;
bool bmpPresent = false;
float groundAltitude = 0;
static SemaphoreHandle_t i2cMutex = nullptr;

struct AltitudeReading {
  float altitude;
  unsigned long timestamp;
};

// Buffer to store altitude readings with timestamps
platform::RingBuffer<AltitudeReading, VARIO_BUFFER_SIZE> altitudeBuffer;

// Publish complete samples so consumers never mix readings from different cycles.
static portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
static BarometerSnapshot cachedSample = {
    0.0f, __FLT_MIN__, __FLT_MIN__, 0.0f, 0, false};

static float computeVerticalSpeed() {
  if (altitudeBuffer.size() < 2) return 0.0f;
  const auto& oldest = altitudeBuffer.first();
  const auto& newest = altitudeBuffer.last();
  const float seconds = (newest.timestamp - oldest.timestamp) / 1000.0f;
  if (seconds <= 0) return 0.0f;
  return platform::clamp((newest.altitude - oldest.altitude) / seconds,
                         -MAX_VERTICAL_SPEED, MAX_VERTICAL_SPEED);
}

BarometerSnapshot getBarometerSnapshot() {
  portENTER_CRITICAL(&snapshotMux);
  const BarometerSnapshot result = cachedSample;
  portEXIT_CRITICAL(&snapshotMux);
  return result;
}

static void publishSample(const BarometerSnapshot& sample) {
  portENTER_CRITICAL(&snapshotMux);
  cachedSample = sample;
  portEXIT_CRITICAL(&snapshotMux);
}

// Caller holds i2cMutex, which also protects the datum and vario ring buffer.
static bool readSample(const STR_DEVICE_DATA_140_V1& deviceData, bool zero) {
  float temperature = 0, pressure = 0;
  if (!bmp.readSample(&temperature, &pressure) ||
      !std::isfinite(deviceData.sea_pressure) || deviceData.sea_pressure <= 0) {
    return false;
  }
  const float absolute = 44330.0f *
      (1.0f - std::pow(pressure / deviceData.sea_pressure, 0.1903f));
  if (zero) {
    groundAltitude = absolute;
    altitudeBuffer.clear();
  }
  const float relative = absolute - groundAltitude;
  const uint32_t now = platform::millis32();
  altitudeBuffer.push({relative, now});
  publishSample({relative, temperature, pressure, computeVerticalSpeed(), now, true});
  return true;
}

bool sampleBarometer(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (!bmpPresent || i2cMutex == nullptr ||
      xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  const bool ok = readSample(deviceData, false);
  xSemaphoreGive(i2cMutex);
  return ok;
}

float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  (void)deviceData;
  const auto sample = getBarometerSnapshot();
  return sample.valid ? sample.altitude : __FLT_MIN__;
}
float getVerticalSpeed() { return getCachedVerticalSpeed(); }

void setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (!bmpPresent || i2cMutex == nullptr ||
      xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  readSample(deviceData, true);
  xSemaphoreGive(i2cMutex);
}

float getBaroTemperature() { return getCachedBaroTemperature(); }
float getBaroPressure() { return getBarometerSnapshot().pressure; }
float getCachedAltitude() { return getBarometerSnapshot().altitude; }
float getCachedBaroTemperature() { return getBarometerSnapshot().temperature; }
float getCachedVerticalSpeed() { return getBarometerSnapshot().verticalSpeed; }

// Start the bmp3XX sensor
bool setupAltimeter() {
  i2cMutex = xSemaphoreCreateMutex();

  // pull down pin 40 to high to set the address
  eppg_gpio_config(40, true, false);
  eppg_gpio_write(40, 1);

  // Give the BMP3xx time to finish powering up after cold boot.
  // Without this, the first begin_I2C() occasionally fails before the
  // sensor is ready, leaving bmpPresent=false for the whole session and
  // tripping the Baro_Init_Failure alert.
  platform::delayMs(20);

  // Retry a few times — most failures here are transient I2C timing races.
  const int kInitAttempts = 4;
  bool ok = false;
  for (int i = 1; i <= kInitAttempts; i++) {
    if (bmp.initialize()) {
      ok = true;
      if (i > 1) {
        platform::console.format("BMP3xx initialized on attempt %d\n", i);
      }
      break;
    }
    platform::console.format("BMP3xx init attempt %d/%d failed\n", i, kInitAttempts);
    platform::delayMs(50);
  }
  if (!ok) return false;

  bmp.readPressure();  // throw away first reading
  // Seed the temperature cache so monitors and telemetry don't see __FLT_MIN__
  // until the first periodic read populates it.
  cachedSample.temperature = bmp.readTemperature();
  bmpPresent = true;
  return true;
}
