#include "platform/barometer.h"
#include "platform/peripheral.h"
#include "esp_rom_sys.h"

namespace platform {
namespace {
BMP3_INTF_RET_TYPE read(uint8_t reg, uint8_t* data, uint32_t length, void*) {
  return eppg_i2c_read(0x77, reg, data, length) == 0 ? BMP3_INTF_RET_SUCCESS : -1;
}
BMP3_INTF_RET_TYPE write(uint8_t reg, const uint8_t* data, uint32_t length, void*) {
  return eppg_i2c_write(0x77, reg, data, length) == 0 ? BMP3_INTF_RET_SUCCESS : -1;
}
void wait(uint32_t us, void*) { esp_rom_delay_us(us); }
bool validCalibration(bmp3_dev* sensor) {
  uint8_t trim[21]{}, stored = 0;
  if (bmp3_get_regs(BMP3_REG_CALIB_DATA, trim, sizeof(trim), sensor) != BMP3_OK ||
      bmp3_get_regs(0x30, &stored, 1, sensor) != BMP3_OK) return false;
  uint8_t crc = 0xff;
  for (uint8_t byte : trim) {
    for (int bit = 0; bit < 8; ++bit) {
      const bool carry = (crc ^ byte) & 0x80;
      crc <<= 1;
      byte <<= 1;
      if (carry) crc ^= 0x1d;
    }
  }
  return stored == static_cast<uint8_t>(crc ^ 0xff);
}
}
bool Barometer::initialize() {
  sensor_ = {};
  sensor_.intf = BMP3_I2C_INTF;
  sensor_.intf_ptr = this;
  sensor_.read = read;
  sensor_.write = write;
  sensor_.delay_us = wait;
  if (bmp3_soft_reset(&sensor_) != BMP3_OK || bmp3_init(&sensor_) != BMP3_OK ||
      !validCalibration(&sensor_)) return false;
  sensor_.settings.temp_en = BMP3_ENABLE;
  sensor_.settings.press_en = BMP3_ENABLE;
  sensor_.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
  sensor_.settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
  sensor_.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_15;
  sensor_.settings.odr_filter.odr = BMP3_ODR_25_HZ;
  sensor_.settings.op_mode = BMP3_MODE_FORCED;
  return true;
}
bool Barometer::sample() {
  const uint16_t settings = BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_EN |
      BMP3_SEL_TEMP_OS | BMP3_SEL_PRESS_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR;
  if (bmp3_set_sensor_settings(settings, &sensor_) != BMP3_OK) return false;
  sensor_.settings.op_mode = BMP3_MODE_FORCED;
  if (bmp3_set_op_mode(&sensor_) != BMP3_OK) return false;
  bmp3_data data{};
  if (bmp3_get_sensor_data(BMP3_TEMP | BMP3_PRESS, &data, &sensor_) != BMP3_OK) return false;
  temperature_ = data.temperature;
  pressure_ = data.pressure;
  return true;
}
bool Barometer::readSample(float* temperature_c, float* pressure_hpa) {
  if (!sample() || !std::isfinite(temperature_) ||
      !std::isfinite(pressure_) || pressure_ <= 0) return false;
  *temperature_c = static_cast<float>(temperature_);
  *pressure_hpa = static_cast<float>(pressure_) / 100.0f;
  return true;
}
float Barometer::readAltitude(float sea_pressure_hpa) {
  sample();
  const float atmospheric = static_cast<float>(pressure_) / 100.0f;
  return 44330.0 * (1.0 - std::pow(atmospheric / sea_pressure_hpa, 0.1903));
}
float Barometer::readTemperature() { sample(); return temperature_; }
float Barometer::readPressure() { sample(); return pressure_; }
}
