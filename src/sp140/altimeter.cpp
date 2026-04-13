#include "sp140/altimeter.h"
#include "sp140/structs.h"

#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
bool bmpPresent = false;
float groundAltitude = 0;

struct AltitudeReading {
  float altitude;
  unsigned long timestamp;
};

// Buffer to store altitude readings with timestamps
CircularBuffer<AltitudeReading, VARIO_BUFFER_SIZE> altitudeBuffer;

// Mutex to serialize I2C barometer access between tasks
static SemaphoreHandle_t i2cBaroMutex = NULL;

// Cached values — written by the designated reader (uiTask via getAltitude),
// safe to read from any task (32-bit float reads are atomic on Xtensa)
static float cachedAltitude = 0.0f;
static float cachedBaroTemperature = __FLT_MIN__;
static float cachedVerticalSpeed = 0.0f;

void initAltimeterMutex() {
  i2cBaroMutex = xSemaphoreCreateMutex();
}

// Compute vertical speed from the altitude buffer (must be called from same
// task that pushes to altitudeBuffer to avoid data races on CircularBuffer)
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
  if (bmpPresent) {
    if (i2cBaroMutex == NULL || xSemaphoreTake(i2cBaroMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      const float altitude = bmp.readAltitude(deviceData.sea_pressure);
      if (i2cBaroMutex != NULL) xSemaphoreGive(i2cBaroMutex);

      float relativeAltitude = altitude - groundAltitude;

      // Add new reading to buffer with timestamp
      AltitudeReading reading = {relativeAltitude, millis()};
      altitudeBuffer.push(reading);

      // Update caches (atomic float writes on Xtensa)
      cachedAltitude = relativeAltitude;
      cachedVerticalSpeed = computeVerticalSpeed();

      return relativeAltitude;
    }
    // Mutex busy — return cached value, skip this cycle
    return cachedAltitude;
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
  if (bmpPresent) {
    if (i2cBaroMutex == NULL || xSemaphoreTake(i2cBaroMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      groundAltitude = bmp.readAltitude(deviceData.sea_pressure);
      altitudeBuffer.clear();
      if (i2cBaroMutex != NULL) xSemaphoreGive(i2cBaroMutex);
    }
  }
}

// Get the temperature in degrees Celsius
float getBaroTemperature() {
  if (bmpPresent) {
    if (i2cBaroMutex == NULL || xSemaphoreTake(i2cBaroMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      float temp = bmp.readTemperature();
      if (i2cBaroMutex != NULL) xSemaphoreGive(i2cBaroMutex);
      cachedBaroTemperature = temp;
      return temp;
    }
    return cachedBaroTemperature;
  }
  return __FLT_MIN__;
}

// Get the pressure in hPa
float getBaroPressure() {
  if (bmpPresent) {
    if (i2cBaroMutex == NULL || xSemaphoreTake(i2cBaroMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      float pressure = bmp.readPressure() / 100.0f;
      if (i2cBaroMutex != NULL) xSemaphoreGive(i2cBaroMutex);
      return pressure;
    }
    return __FLT_MIN__;
  }
  return __FLT_MIN__;
}

// Cached getters — safe to call from any task
float getCachedAltitude() { return cachedAltitude; }
float getCachedBaroTemperature() { return cachedBaroTemperature; }
float getCachedVerticalSpeed() { return cachedVerticalSpeed; }

// Start the bmp3XX sensor
bool setupAltimeter() {
  // pull down pin 40 to high to set the address
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  if (!bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire)) return false;

  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.readPressure();  // throw away first reading
  bmpPresent = true;
  return true;
}
