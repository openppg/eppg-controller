#include "platform/peripheral.h"
#include "platform/runtime.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/uart.h"
#include "driver/temperature_sensor.h"
#include "driver/rmt_tx.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#ifdef EPPG_QEMU
extern "C" {
int eppg_qemu_gpio_read(int pin);
void eppg_qemu_gpio_write(int pin, int value);
int eppg_qemu_adc_read(int pin);
int eppg_qemu_spi_transfer(void*, const uint8_t*, uint8_t*, size_t);
int eppg_qemu_i2c_read(uint8_t, uint8_t, uint8_t*, size_t);
int eppg_qemu_i2c_write(uint8_t, uint8_t, const uint8_t*, size_t);
void eppg_qemu_pwm_config(int, int, uint32_t);
void eppg_qemu_pwm_write(int, uint32_t);
void eppg_qemu_pwm_frequency(int, uint32_t);
void eppg_qemu_pixel_write(int, uint32_t);
void eppg_qemu_console_start();
int eppg_qemu_usb_read(uint8_t*, size_t);
}
#endif

extern "C" uint32_t eppg_millis() { return esp_timer_get_time() / 1000; }
extern "C" uint64_t eppg_micros() { return esp_timer_get_time(); }
extern "C" void eppg_delay_ms(uint32_t ms) {
  if (ms) vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(ms)));
  else taskYIELD();
}
extern "C" int eppg_gpio_config(int pin, bool output, bool pullup) {
#ifdef EPPG_QEMU
  return ESP_OK;
#else
  gpio_config_t config{};
  config.pin_bit_mask = 1ULL << pin;
  config.mode = output ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
  config.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  return gpio_config(&config);
#endif
}
extern "C" int eppg_gpio_read(int pin) {
#ifdef EPPG_QEMU
  return eppg_qemu_gpio_read(pin);
#else
  return gpio_get_level(static_cast<gpio_num_t>(pin));
#endif
}
extern "C" void eppg_gpio_write(int pin, int value) {
#ifdef EPPG_QEMU
  eppg_qemu_gpio_write(pin, value);
#else
  gpio_set_level(static_cast<gpio_num_t>(pin), value);
#endif
}
extern "C" int eppg_adc_read(int pin) {
#ifdef EPPG_QEMU
  return eppg_qemu_adc_read(pin);
#else
  static adc_oneshot_unit_handle_t adc = nullptr;
  static int configured_pin = -1;
  adc_unit_t unit;
  adc_channel_t channel;
  if (adc_oneshot_io_to_channel(pin, &unit, &channel) != ESP_OK || unit != ADC_UNIT_1) return 0;
  if (!adc) {
    adc_oneshot_unit_init_cfg_t config{};
    config.unit_id = unit;
    if (adc_oneshot_new_unit(&config, &adc) != ESP_OK) return 0;
  }
  if (configured_pin != pin) {
    adc_oneshot_chan_cfg_t config{};
    config.atten = ADC_ATTEN_DB_12;
    config.bitwidth = ADC_BITWIDTH_12;
    if (adc_oneshot_config_channel(adc, channel, &config) != ESP_OK) return 0;
    configured_pin = pin;
  }
  int raw = 0;
  return adc_oneshot_read(adc, channel, &raw) == ESP_OK ? raw : 0;
#endif
}
extern "C" int eppg_pwm_config(int channel, int pin, uint32_t hz) {
#ifdef EPPG_QEMU
  eppg_qemu_pwm_config(channel, pin, hz);
  return ESP_OK;
#else
  // Each feedback output owns a timer; changing a note never changes vibration.
  ledc_timer_config_t timer{};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = static_cast<ledc_timer_t>(channel);
  timer.freq_hz = hz;
  timer.clk_cfg = LEDC_AUTO_CLK;
  esp_err_t err = ledc_timer_config(&timer);
  if (err != ESP_OK) return err;
  ledc_channel_config_t output{};
  output.gpio_num = pin;
  output.speed_mode = LEDC_LOW_SPEED_MODE;
  output.channel = static_cast<ledc_channel_t>(channel);
  output.timer_sel = timer.timer_num;
  output.duty = 0;
  return ledc_channel_config(&output);
#endif
}
extern "C" void eppg_pwm_write(int channel, uint32_t duty) {
#ifdef EPPG_QEMU
  eppg_qemu_pwm_write(channel, duty);
#else
  ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel), std::min<uint32_t>(duty, 255));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel));
#endif
}
extern "C" void eppg_pwm_frequency(int channel, uint32_t hz) {
#ifdef EPPG_QEMU
  eppg_qemu_pwm_frequency(channel, hz);
#else
  ledc_set_freq(LEDC_LOW_SPEED_MODE, static_cast<ledc_timer_t>(channel), hz);
#endif
}
extern "C" int eppg_pixel_write(int pin, uint32_t rgb) {
#ifdef EPPG_QEMU
  eppg_qemu_pixel_write(pin, rgb);
  return ESP_OK;
#else
  static rmt_channel_handle_t channel = nullptr;
  static rmt_encoder_handle_t encoder = nullptr;
  if (!channel) {
    rmt_tx_channel_config_t config{};
    config.gpio_num = static_cast<gpio_num_t>(pin);
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = 10000000;
    config.mem_block_symbols = 64;
    config.trans_queue_depth = 1;
    esp_err_t err = rmt_new_tx_channel(&config, &channel);
    if (err != ESP_OK) return err;
    rmt_copy_encoder_config_t encoding{};
    err = rmt_new_copy_encoder(&encoding, &encoder);
    if (err != ESP_OK) return err;
    rmt_enable(channel);
  }
  // WS2812 GRB, 800 kHz; latch low after the 24-bit sequence.
  uint8_t colors[] = {static_cast<uint8_t>(rgb >> 8), static_cast<uint8_t>(rgb >> 16), static_cast<uint8_t>(rgb)};
  rmt_symbol_word_t symbols[25]{};
  for (int bit = 0; bit < 24; ++bit) {
    const bool one = colors[bit / 8] & (0x80 >> (bit % 8));
    symbols[bit].level0 = 1;
    symbols[bit].duration0 = one ? 8 : 4;
    symbols[bit].level1 = 0;
    symbols[bit].duration1 = one ? 4 : 8;
  }
  symbols[24].duration0 = 500;
  symbols[24].duration1 = 500;
  rmt_transmit_config_t tx{};
  esp_err_t err = rmt_transmit(channel, encoder, symbols, sizeof(symbols), &tx);
  return err == ESP_OK ? rmt_tx_wait_all_done(channel, 100) : err;
#endif
}
extern "C" int eppg_spi_bus_init(int mosi, int miso, int sclk) {
#ifdef EPPG_QEMU
  return ESP_OK;
#else
  spi_bus_config_t bus{};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = sclk;
  bus.quadwp_io_num = bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 40960;
  return spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
#endif
}
extern "C" void* eppg_spi_add_device(int cs, uint32_t hz, uint8_t mode) {
#ifdef EPPG_QEMU
  return reinterpret_cast<void*>(static_cast<uintptr_t>(cs + 1));
#else
  spi_device_interface_config_t config{};
  config.clock_speed_hz = hz;
  config.mode = mode;
  config.spics_io_num = cs;
  config.queue_size = 1;
  spi_device_handle_t device = nullptr;
  return spi_bus_add_device(SPI2_HOST, &config, &device) == ESP_OK ? device : nullptr;
#endif
}
extern "C" int eppg_spi_transfer(void* device, const uint8_t* tx, uint8_t* rx, size_t length) {
#ifdef EPPG_QEMU
  return eppg_qemu_spi_transfer(device, tx, rx, length);
#else
  if (!device) return ESP_ERR_INVALID_STATE;
  spi_transaction_t transaction{};
  transaction.length = length * 8;
  transaction.tx_buffer = tx;
  transaction.rx_buffer = rx;
  return spi_device_polling_transmit(static_cast<spi_device_handle_t>(device), &transaction);
#endif
}
#ifndef EPPG_QEMU
namespace {
i2c_master_bus_handle_t sensor_bus = nullptr;
std::array<i2c_master_dev_handle_t, 128> sensor_devices{};
// Application I2C belongs to the single barometer owner (UI in C++, sensor task
// in Rust). Cache device handles; synchronous transactions are serialized by IDF.
esp_err_t sensor_device(uint8_t address, i2c_master_dev_handle_t* out) {
  if (!sensor_bus || address >= sensor_devices.size()) return ESP_ERR_INVALID_ARG;
  if (!sensor_devices[address]) {
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = address;
    config.scl_speed_hz = 100000;
    const esp_err_t err = i2c_master_bus_add_device(sensor_bus, &config, &sensor_devices[address]);
    if (err != ESP_OK) return err;
  }
  *out = sensor_devices[address];
  return ESP_OK;
}
}
#endif
extern "C" int eppg_i2c_init(int sda, int scl) {
#ifdef EPPG_QEMU
  return ESP_OK;
#else
  if (sensor_bus) return ESP_ERR_INVALID_STATE;
  i2c_master_bus_config_t config{};
  config.i2c_port = I2C_NUM_0;
  config.sda_io_num = static_cast<gpio_num_t>(sda);
  config.scl_io_num = static_cast<gpio_num_t>(scl);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;
  return i2c_new_master_bus(&config, &sensor_bus);
#endif
}
extern "C" int eppg_i2c_read(uint8_t address, uint8_t reg, uint8_t* data, size_t length) {
#ifdef EPPG_QEMU
  return eppg_qemu_i2c_read(address, reg, data, length);
#else
  i2c_master_dev_handle_t device = nullptr;
  const esp_err_t err = sensor_device(address, &device);
  return err == ESP_OK ? i2c_master_transmit_receive(device, &reg, 1, data, length, 50) : err;
#endif
}
extern "C" int eppg_i2c_write(uint8_t address, uint8_t reg, const uint8_t* data, size_t length) {
#ifdef EPPG_QEMU
  return eppg_qemu_i2c_write(address, reg, data, length);
#else
  i2c_master_dev_handle_t device = nullptr;
  const esp_err_t err = sensor_device(address, &device);
  if (err != ESP_OK) return err;
  std::vector<uint8_t> packet(length + 1);
  packet[0] = reg;
  memcpy(packet.data() + 1, data, length);
  return i2c_master_transmit(device, packet.data(), packet.size(), 50);
#endif
}
extern "C" int eppg_usb_init() {
#ifdef EPPG_QEMU
  uart_config_t config{};
  config.baud_rate = 115200;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_param_config(UART_NUM_0, &config);
  const esp_err_t err = uart_driver_install(UART_NUM_0, 2048, 2048, 0, nullptr, 0);
  if (err == ESP_OK) eppg_qemu_console_start();
  return err;
#else
  usb_serial_jtag_driver_config_t config{};
  config.rx_buffer_size = config.tx_buffer_size = 2048;
  const esp_err_t err = usb_serial_jtag_driver_install(&config);
  // SDK stdout and application USB traffic must use the same interrupt-driven
  // transport rather than concurrently touching the hardware FIFO.
  if (err == ESP_OK) usb_serial_jtag_vfs_use_driver();
  return err;
#endif
}
extern "C" int eppg_usb_read(uint8_t* data, size_t length) {
#ifdef EPPG_QEMU
  return eppg_qemu_usb_read(data, length);
#else
  return usb_serial_jtag_read_bytes(data, length, 0);
#endif
}
extern "C" int eppg_usb_write(const uint8_t* data, size_t length) {
#ifdef EPPG_QEMU
  return uart_write_bytes(UART_NUM_0, data, length);
#else
  return usb_serial_jtag_write_bytes(data, length, pdMS_TO_TICKS(20));
#endif
}
extern "C" float eppg_cpu_temperature() {
#ifdef EPPG_QEMU
  return 30.0f;
#else
  static temperature_sensor_handle_t handle = nullptr;
  if (!handle) {
    temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&config, &handle) != ESP_OK) return NAN;
    temperature_sensor_enable(handle);
  }
  float value = NAN;
  temperature_sensor_get_celsius(handle, &value);
  return value;
#endif
}

namespace platform {
Console console;
bool Console::hasInput() {
  if (pending_ < 0) {
    uint8_t byte;
    if (eppg_usb_read(&byte, 1) == 1) pending_ = byte;
  }
  return pending_ >= 0;
}
int Console::readByte() {
  if (!hasInput()) return -1;
  const int value = pending_;
  pending_ = -1;
  return value;
}
}
