#include "sp140/altimeter.h"
#include "sp140/structs.h"

#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
bool bmpPresent = false;
float groundAltitude = 0;
static SemaphoreHandle_t i2cMutex = nullptr;

struct AltitudeReading {
  float altitude;
  unsigned long timestamp;
};

// Buffer to store altitude readings with timestamps
CircularBuffer<AltitudeReading, VARIO_BUFFER_SIZE> altitudeBuffer;

// Cached values — written by the designated I2C reader (uiTask via getAltitude),
// safe to read from any task (32-bit float reads are atomic on Xtensa).
static float cachedAltitude = 0.0f;
static float cachedBaroTemperature = __FLT_MIN__;
static float cachedVerticalSpeed = 0.0f;

// Recompute vertical speed from the altitude buffer. Must be called from the
// same task that pushes to altitudeBuffer (uiTask) to avoid CircularBuffer races.
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

float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (bmpPresent && i2cMutex != nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      const float altitude = bmp.readAltitude(deviceData.sea_pressure);
      xSemaphoreGive(i2cMutex);
      float relativeAltitude = altitude - groundAltitude;

      // Add new reading to buffer with timestamp
      AltitudeReading reading = {relativeAltitude, millis()};
      altitudeBuffer.push(reading);

      // Update caches (atomic 32-bit float writes on Xtensa)
      cachedAltitude = relativeAltitude;
      cachedVerticalSpeed = computeVerticalSpeed();

      return relativeAltitude;
    }
  }
  return __FLT_MIN__;
}

float getVerticalSpeed() {
  if (altitudeBuffer.size() < 2) {
    return 0.0f;  // Not enough readings yet
  }

  // Get oldest and newest readings
  const AltitudeReading& oldest = altitudeBuffer.first();
  const AltitudeReading& newest = altitudeBuffer.last();

  // Calculate time difference in seconds
  float timeDiff = (newest.timestamp - oldest.timestamp) / 1000.0f;
  if (timeDiff <= 0) {
    return 0.0f;  // Avoid division by zero
  }

  // Calculate vertical speed in meters per second
  float verticalSpeed = (newest.altitude - oldest.altitude) / timeDiff;

  // Constrain to max vertical speed
  return constrain(verticalSpeed, -MAX_VERTICAL_SPEED, MAX_VERTICAL_SPEED);
}

// set the ground altitude to the current altitude
void setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (bmpPresent && i2cMutex != nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      groundAltitude = bmp.readAltitude(deviceData.sea_pressure);
      xSemaphoreGive(i2cMutex);
      altitudeBuffer.clear();  // Clear the buffer when resetting ground altitude
    }
  }
}

// Get the temperature in degrees Celsius
float getBaroTemperature() {
  if (bmpPresent && i2cMutex != nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      float temp = bmp.readTemperature();
      xSemaphoreGive(i2cMutex);
      cachedBaroTemperature = temp;  // Update cache for cross-task reads
      return temp;
    }
  }
  return __FLT_MIN__;
}

// Cached getters — safe to call from any task (atomic float reads on Xtensa).
float getCachedAltitude() { return cachedAltitude; }
float getCachedBaroTemperature() { return cachedBaroTemperature; }
float getCachedVerticalSpeed() { return cachedVerticalSpeed; }

// Get the pressure in hPa
float getBaroPressure() {
  if (bmpPresent && i2cMutex != nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      float pressure = bmp.readPressure() / 100.0f;
      xSemaphoreGive(i2cMutex);
      return pressure;
    }
  }
  return __FLT_MIN__;
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
  bmp.readPressure();  // throw away first reading
  // Seed the temperature cache so monitors and telemetry don't see __FLT_MIN__
  // until the first periodic read populates it.
  cachedBaroTemperature = bmp.readTemperature();
  bmpPresent = true;
  return true;
}
