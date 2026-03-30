#include "sp140/altimeter.h"
#include "sp140/structs.h"

#include <Adafruit_BMP3XX.h>
#include <cmath>

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

float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (bmpPresent && i2cMutex != nullptr) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      const float altitude = bmp.readAltitude(deviceData.sea_pressure);
      xSemaphoreGive(i2cMutex);
      float relativeAltitude = altitude - groundAltitude;

      // Add new reading to buffer with timestamp
      AltitudeReading reading = {relativeAltitude, millis()};
      altitudeBuffer.push(reading);

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

  // Guard against NaN/Inf altitude readings contaminating vario
  if (!isfinite(newest.altitude) || !isfinite(oldest.altitude)) {
    return 0.0f;
  }

  // Calculate time difference in seconds
  float timeDiff = (newest.timestamp - oldest.timestamp) / 1000.0f;
  if (timeDiff <= 0) {
    return 0.0f;  // Avoid division by zero
  }

  // Calculate vertical speed in meters per second
  float verticalSpeed = (newest.altitude - oldest.altitude) / timeDiff;

  // Final NaN guard (belt-and-suspenders) and constrain to max vertical speed
  if (!isfinite(verticalSpeed)) {
    return 0.0f;
  }
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
      return temp;
    }
  }
  return __FLT_MIN__;
}

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
  if (!bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire)) return false;

  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.readPressure();  // throw away first reading
  bmpPresent = true;
  return true;
}
