#include "sp140/altimeter.h"
#include "sp140/structs.h"

#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
bool bmpPresent = false;
float groundAltitude = 0;

// Buffer to store altitude readings with timestamps
struct AltitudeReading {
  float altitude;
  unsigned long timestamp;
};

CircularBuffer<AltitudeReading, VARIO_BUFFER_SIZE> altitudeBuffer;

float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (bmpPresent) {
    const float altitude = bmp.readAltitude(deviceData.sea_pressure);
    float relativeAltitude = altitude - groundAltitude;

    // Add new reading to buffer with timestamp
    AltitudeReading reading = {relativeAltitude, millis()};
    altitudeBuffer.push(reading);

    return relativeAltitude;
  }
  return __FLT_MIN__;
}

float getVerticalSpeed() {
  if (altitudeBuffer.size() < 2) {
    return 0.0f;  // Not enough readings yet
  }

  // Calculate 5-second rolling average
  const unsigned long currentTime = millis();
  const unsigned long fiveSecondsAgo = currentTime - 5000;  // 5 seconds ago
  
  // Find the oldest reading within the last 5 seconds
  int oldestIndex = -1;
  for (int i = 0; i < altitudeBuffer.size(); i++) {
    if (altitudeBuffer[i].timestamp >= fiveSecondsAgo) {
      oldestIndex = i;
      break;
    }
  }
  
  // If no readings within 5 seconds, use the oldest available reading
  if (oldestIndex == -1) {
    oldestIndex = 0;
  }
  
  // Get the readings for the 5-second window
  const AltitudeReading& oldest = altitudeBuffer[oldestIndex];
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
    groundAltitude = bmp.readAltitude(deviceData.sea_pressure);
    altitudeBuffer.clear();  // Clear the buffer when resetting ground altitude
  }
}

// Get the temperature in degrees Celsius
float getBaroTemperature() {
  if (bmpPresent) {
    return bmp.readTemperature();
  }
  return __FLT_MIN__;  // Return a very small number if BMP is not present
}

// Start the bmp3XX sensor
bool setupAltimeter(bool altWire) {
  TwoWire* wire = &Wire;

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
