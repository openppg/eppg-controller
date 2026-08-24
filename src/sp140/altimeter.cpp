#include "sp140/altimeter.h"
#include "sp140/structs.h"

#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
bool bmpPresent = false;
float groundAltitude = 0;
static SemaphoreHandle_t i2cMutex = nullptr;
static portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;

struct AltitudeReading {
  float altitude;
  unsigned long timestamp;
};

// Buffer to store altitude readings with timestamps
CircularBuffer<AltitudeReading, VARIO_BUFFER_SIZE> altitudeBuffer;

// Readers copy this entire structure inside a very short cross-core critical
// section. Independent atomic float loads would still permit mixed samples.
static BarometerSnapshot cachedSample = {
    0.0f, 0.0f, __FLT_MIN__, __FLT_MIN__, 0.0f, 0, false};

// Recompute vertical speed from the altitude buffer. Must be called from the
// same task that pushes to altitudeBuffer to avoid CircularBuffer races.
static float computeVerticalSpeed() {
  if (altitudeBuffer.size() < 2) {
    return 0.0f;
  }
  const AltitudeReading& oldest = altitudeBuffer.first();
  const AltitudeReading& newest = altitudeBuffer.last();
  float timeDiff = (newest.timestamp - oldest.timestamp) / 1000.0f;
  if (timeDiff <= 0) {
    return 0.0f;
  }
  float verticalSpeed = (newest.altitude - oldest.altitude) / timeDiff;
  return constrain(verticalSpeed, -MAX_VERTICAL_SPEED, MAX_VERTICAL_SPEED);
}

static void publishSnapshot(const BarometerSnapshot& snapshot) {
  portENTER_CRITICAL(&snapshotMux);
  cachedSample = snapshot;
  portEXIT_CRITICAL(&snapshotMux);
}

bool getBarometerSnapshot(BarometerSnapshot* snapshot) {
  if (snapshot == nullptr) return false;

  portENTER_CRITICAL(&snapshotMux);
  *snapshot = cachedSample;
  portEXIT_CRITICAL(&snapshotMux);
  return snapshot->valid;
}

bool sampleBarometer(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (bmpPresent && i2cMutex != nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (!bmp.performReading()) {
        xSemaphoreGive(i2cMutex);
        return false;
      }

      const float pressureHpa = bmp.pressure / 100.0f;
      const float absoluteAltitude =
          44330.0f *
          (1.0f - pow(pressureHpa / deviceData.sea_pressure, 0.1903f));
      const float relativeAltitude = absoluteAltitude - groundAltitude;
      const uint32_t timestampMs = millis();

      // Add new reading to buffer with timestamp
      AltitudeReading reading = {relativeAltitude, timestampMs};
      altitudeBuffer.push(reading);

      BarometerSnapshot next = {
          relativeAltitude, absoluteAltitude,
          static_cast<float>(bmp.temperature), pressureHpa,
          computeVerticalSpeed(), timestampMs, true};
      publishSnapshot(next);
      xSemaphoreGive(i2cMutex);

      return true;
    }
  }
  return false;
}

float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  (void)deviceData;
  BarometerSnapshot snapshot = {};
  return getBarometerSnapshot(&snapshot) ? snapshot.relativeAltitude
                                         : __FLT_MIN__;
}

float getVerticalSpeed() { return getCachedVerticalSpeed(); }

// Set ground altitude from the same sample consumers have observed. Never force
// a conversion during the arm transition, but reject missing or stale data.
bool setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  (void)deviceData;
  if (bmpPresent && i2cMutex != nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      BarometerSnapshot snapshot = {};
      const bool valid = getBarometerSnapshot(&snapshot);
      const bool fresh = valid &&
          static_cast<uint32_t>(millis() - snapshot.sampledAtMs) <=
              BAROMETER_ARM_MAX_AGE_MS;
      if (!fresh) {
        xSemaphoreGive(i2cMutex);
        return false;
      }

      groundAltitude = snapshot.absoluteAltitude;
      snapshot.relativeAltitude = 0.0f;
      snapshot.verticalSpeedMps = 0.0f;
      publishSnapshot(snapshot);
      altitudeBuffer.clear();
      xSemaphoreGive(i2cMutex);
      return true;
    }
  }
  return false;
}

// Get the temperature in degrees Celsius
float getBaroTemperature() {
  return getCachedBaroTemperature();
}

float getCachedAltitude() {
  BarometerSnapshot snapshot = {};
  return getBarometerSnapshot(&snapshot) ? snapshot.relativeAltitude : 0.0f;
}

float getCachedBaroTemperature() {
  BarometerSnapshot snapshot = {};
  return getBarometerSnapshot(&snapshot) ? snapshot.temperatureC : __FLT_MIN__;
}

float getCachedBaroPressure() {
  BarometerSnapshot snapshot = {};
  return getBarometerSnapshot(&snapshot) ? snapshot.pressureHpa : __FLT_MIN__;
}

float getCachedVerticalSpeed() {
  BarometerSnapshot snapshot = {};
  return getBarometerSnapshot(&snapshot) ? snapshot.verticalSpeedMps : 0.0f;
}

// Get the pressure in hPa
float getBaroPressure() {
  return getCachedBaroPressure();
}

// Start the bmp3XX sensor
bool setupAltimeter() {
  i2cMutex = xSemaphoreCreateMutex();

  // pull down pin 40 to high to set the address
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);

  // Give the BMP3xx time to finish powering up after cold boot.
  // Without this, the first begin_I2C() occasionally fails before the
  // sensor is ready, leaving bmpPresent=false for the whole session and
  // tripping the Baro_Init_Failure alert.
  delay(20);

  // Retry a few times — most failures here are transient I2C timing races.
  const int kInitAttempts = 4;
  bool ok = false;
  for (int i = 1; i <= kInitAttempts; i++) {
    if (bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire)) {
      ok = true;
      if (i > 1) {
        USBSerial.printf("BMP3xx initialized on attempt %d\n", i);
      }
      break;
    }
    USBSerial.printf("BMP3xx init attempt %d/%d failed\n", i, kInitAttempts);
    delay(50);
  }
  if (!ok) return false;

  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmpPresent = true;
  return true;
}
