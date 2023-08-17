#include "sp140/altimeter.h"
#include "sp140/structs.h"

#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
bool bmpPresent = false;
float groundAltitude = 0;

float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (bmpPresent) {
    const float altitude = bmp.readAltitude(deviceData.sea_pressure);
    return altitude - groundAltitude;
  }
  return __FLT_MIN__;
}

// set the ground altitude to the current altitude
void setGroundAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  groundAltitude = bmp.readAltitude(deviceData.sea_pressure);
}

// Start the bmp388 sensor
void setupAltimeter() {
  if (!bmp.begin_I2C()) return;
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.readPressure();  // throw away first reading
  bmpPresent = true;
}