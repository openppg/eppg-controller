#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace eppg::sim::devices {

struct CanFrame {
  uint32_t id = 0;
  uint8_t length = 0;
  bool extended = true;
  bool remote = false;
  std::array<uint8_t, 8> data{};
};

// Register-level subset used by the controller's MCP2515 SPI driver. The two
// receive buffers are intentionally finite: a third frame raises overflow.
class Mcp2515 {
 public:
  Mcp2515();
  void reset();
  void transfer(const uint8_t* tx, uint8_t* rx, size_t length);
  bool receive(const CanFrame& frame);
  bool take_transmitted(CanFrame* frame);
  uint8_t reg(uint8_t address) const { return registers_[address]; }
  uint64_t overruns() const { return overruns_; }
  uint64_t transactions() const { return transactions_; }

 private:
  void write(uint8_t address, uint8_t value);
  void request_to_send(unsigned buffer);
  static void encode(const CanFrame& frame, uint8_t* destination);
  static CanFrame decode(const uint8_t* source);
  std::array<uint8_t, 256> registers_{};
  std::array<CanFrame, 3> transmitted_{};
  size_t tx_head_ = 0;
  size_t tx_size_ = 0;
  uint64_t overruns_ = 0;
  uint64_t transactions_ = 0;
};

// ST7735 command/data decoder. Pixels use RGB565 and the configured address
// window; framebuffer contents come from SPI traffic rather than UI snapshots.
class St7735 {
 public:
  static constexpr size_t kPixels = 160 * 128;
  void transfer(bool data, const uint8_t* bytes, size_t length);
  unsigned width() const { return (madctl_ & 0x20) ? 160 : 128; }
  unsigned height() const { return (madctl_ & 0x20) ? 128 : 160; }
  uint16_t pixel(unsigned x, unsigned y) const;
  const std::array<uint16_t, kPixels>& pixels() const { return pixels_; }
  uint64_t pixel_writes() const { return pixel_writes_; }
  uint64_t invalid_windows() const { return invalid_windows_; }
  uint8_t command() const { return command_; }
  bool on() const { return display_on_; }

 private:
  void data_byte(uint8_t byte);
  std::array<uint16_t, kPixels> pixels_{};
  std::array<uint8_t, 4> parameters_{};
  uint16_t x0_ = 0, x1_ = 127, y0_ = 0, y1_ = 159;
  uint16_t x_ = 0, y_ = 0;
  uint8_t command_ = 0, parameter_count_ = 0, high_byte_ = 0;
  uint8_t madctl_ = 0;
  bool have_high_byte_ = false, display_on_ = false;
  uint64_t pixel_writes_ = 0, invalid_windows_ = 0;
};

// BMP388/390 I2C register fixture. Compensation is performed by the real
// controller driver; the emulator supplies raw ADC and calibration registers.
class Bmp3xx {
 public:
  Bmp3xx();
  bool read(uint8_t address, uint8_t* data, size_t length) const;
  bool write(uint8_t address, const uint8_t* data, size_t length);
  void raw_sample(uint32_t pressure_adc, uint32_t temperature_adc);
  void set_connected(bool connected) { connected_ = connected; }
  void set_chip_id(uint8_t id) { registers_[0] = id; }

 private:
  std::array<uint8_t, 256> registers_{};
  bool connected_ = true;
};

// NOR semantics for host OTA/NVS fault injection. It cannot set a programmed
// zero back to one without an erase. Power loss can be injected after N bytes.
class NorFlash {
 public:
  explicit NorFlash(size_t size);
  bool read(size_t offset, uint8_t* data, size_t length) const;
  bool program(size_t offset, const uint8_t* data, size_t length);
  bool erase(size_t offset, size_t length);
  void cut_power_after(size_t bytes) { write_budget_ = bytes; }
  void restore_power() { write_budget_ = static_cast<size_t>(-1); }
  size_t size() const { return size_; }
  ~NorFlash();
  NorFlash(const NorFlash&) = delete;
  NorFlash& operator=(const NorFlash&) = delete;

 private:
  uint8_t* bytes_;
  size_t size_;
  size_t write_budget_ = static_cast<size_t>(-1);
};

}  // namespace eppg::sim::devices
