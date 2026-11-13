#include "peripherals.h"

#include <algorithm>
#include <cstring>

namespace eppg::sim::devices {

Mcp2515::Mcp2515() { reset(); }
void Mcp2515::reset() {
  registers_.fill(0);
  registers_[0x0e] = registers_[0x0f] = 0x80;  // configuration mode
  tx_head_ = tx_size_ = 0;
}
void Mcp2515::write(uint8_t address, uint8_t value) {
  registers_[address] = value;
  if (address == 0x0f) {
    registers_[0x0e] = (registers_[0x0e] & 0x1f) | (value & 0xe0);
  }
}
void Mcp2515::encode(const CanFrame& frame, uint8_t* out) {
  std::fill_n(out, 13, 0);
  if (frame.extended) {
    out[0] = static_cast<uint8_t>(frame.id >> 21);
    out[1] = static_cast<uint8_t>(((frame.id >> 13) & 0xe0) |
                                ((frame.id >> 16) & 3) | 8);
    out[2] = static_cast<uint8_t>(frame.id >> 8);
    out[3] = static_cast<uint8_t>(frame.id);
  } else {
    out[0] = static_cast<uint8_t>(frame.id >> 3);
    out[1] = static_cast<uint8_t>((frame.id & 7) << 5);
  }
  out[4] = std::min<uint8_t>(frame.length, 8) | (frame.remote ? 0x40 : 0);
  std::copy_n(frame.data.begin(), std::min<uint8_t>(frame.length, 8), out + 5);
}
CanFrame Mcp2515::decode(const uint8_t* in) {
  CanFrame frame;
  frame.extended = (in[1] & 8) != 0;
  frame.remote = (in[4] & 0x40) != 0;
  frame.length = std::min<uint8_t>(in[4] & 0x0f, 8);
  frame.id = frame.extended
      ? (uint32_t(in[0]) << 21) | (uint32_t(in[1] & 0xe0) << 13) |
        (uint32_t(in[1] & 3) << 16) | (uint32_t(in[2]) << 8) | in[3]
      : (uint32_t(in[0]) << 3) | (in[1] >> 5);
  std::copy_n(in + 5, frame.length, frame.data.begin());
  return frame;
}
bool Mcp2515::receive(const CanFrame& frame) {
  // Accept frames only in normal or loopback mode.
  if ((registers_[0x0e] & 0xe0) != 0 && (registers_[0x0e] & 0xe0) != 0x40) {
    return false;
  }
  for (unsigned n = 0; n < 2; ++n) {
    if (!(registers_[0x2c] & (1 << n))) {
      encode(frame, registers_.data() + (n ? 0x71 : 0x61));
      registers_[0x2c] |= uint8_t(1 << n);
      return true;
    }
  }
  registers_[0x2d] |= 0xc0;
  ++overruns_;
  return false;
}
void Mcp2515::request_to_send(unsigned buffer) {
  const CanFrame frame = decode(registers_.data() + 0x31 + buffer * 0x10);
  if (tx_size_ < transmitted_.size()) {
    transmitted_[(tx_head_ + tx_size_) % transmitted_.size()] = frame;
    ++tx_size_;
  }
  registers_[0x30 + buffer * 0x10] &= ~uint8_t(8);
  registers_[0x2c] |= uint8_t(4 << buffer);
  if ((registers_[0x0e] & 0xe0) == 0x40) receive(frame);
}
bool Mcp2515::take_transmitted(CanFrame* frame) {
  if (!frame || !tx_size_) return false;
  *frame = transmitted_[tx_head_];
  tx_head_ = (tx_head_ + 1) % transmitted_.size();
  --tx_size_;
  return true;
}
void Mcp2515::transfer(const uint8_t* tx, uint8_t* rx, size_t length) {
  ++transactions_;
  if (rx) std::fill_n(rx, length, 0);
  if (!tx || !length) return;
  const uint8_t command = tx[0];
  if (command == 0xc0) { reset(); return; }
  if (command == 3 && length >= 2) {
    for (size_t i = 2; i < length; ++i) {
      if (rx) rx[i] = registers_[uint8_t(tx[1] + i - 2)];
    }
  } else if (command == 2 && length >= 2) {
    for (size_t i = 2; i < length; ++i) write(uint8_t(tx[1] + i - 2), tx[i]);
  } else if (command == 5 && length >= 4) {
    write(tx[1], (registers_[tx[1]] & ~tx[2]) | (tx[3] & tx[2]));
  } else if (command == 0xa0) {
    const uint8_t flags = registers_[0x2c];
    const uint8_t status = (flags & 3) | ((flags & 4) << 1) |
                           ((flags & 8) << 2) | ((flags & 16) << 3);
    if (rx) for (size_t i = 1; i < length; ++i) rx[i] = status;
  } else if ((command & 0xfc) == 0x90) {
    const unsigned n = (command >> 1) & 1;
    const uint8_t start = (n ? 0x71 : 0x61) + ((command & 1) ? 5 : 0);
    if (rx) for (size_t i = 1; i < length; ++i) rx[i] = registers_[uint8_t(start + i - 1)];
    registers_[0x2c] &= ~uint8_t(1 << n);
  } else if ((command & 0xf8) == 0x80) {
    for (unsigned n = 0; n < 3; ++n) if (command & (1 << n)) request_to_send(n);
  } else if ((command & 0xf8) == 0x40) {
    const unsigned n = (command >> 1) & 3;
    if (n < 3) {
      const uint8_t start = 0x31 + n * 0x10 + ((command & 1) ? 5 : 0);
      for (size_t i = 1; i < length; ++i) write(uint8_t(start + i - 1), tx[i]);
    }
  }
}

uint16_t St7735::pixel(unsigned x, unsigned y) const {
  return x < width() && y < height() ? pixels_[y * width() + x] : 0;
}
void St7735::transfer(bool data, const uint8_t* bytes, size_t length) {
  if (!bytes) return;
  for (size_t i = 0; i < length; ++i) {
    if (data) { data_byte(bytes[i]); continue; }
    command_ = bytes[i];
    parameter_count_ = 0;
    have_high_byte_ = false;
    if (command_ == 1) {
      madctl_ = 0; display_on_ = false;
      x0_ = y0_ = 0; x1_ = 127; y1_ = 159;
    } else if (command_ == 0x28) display_on_ = false;
    else if (command_ == 0x29) display_on_ = true;
    else if (command_ == 0x2c) {
      x_ = x0_; y_ = y0_;
      if (x0_ > x1_ || y0_ > y1_ || x1_ >= width() || y1_ >= height()) ++invalid_windows_;
    }
  }
}
void St7735::data_byte(uint8_t byte) {
  if (command_ == 0x36) { madctl_ = byte; return; }
  if (command_ == 0x2a || command_ == 0x2b) {
    parameters_[parameter_count_++ % 4] = byte;
    if (parameter_count_ == 4) {
      const uint16_t first = (parameters_[0] << 8) | parameters_[1];
      const uint16_t last = (parameters_[2] << 8) | parameters_[3];
      if (command_ == 0x2a) { x0_ = first; x1_ = last; }
      else { y0_ = first; y1_ = last; }
    }
    return;
  }
  if (command_ != 0x2c) return;
  if (!have_high_byte_) { high_byte_ = byte; have_high_byte_ = true; return; }
  const uint16_t color = (uint16_t(high_byte_) << 8) | byte;
  have_high_byte_ = false;
  if (x_ < width() && y_ < height() && x0_ <= x1_ && y0_ <= y1_) {
    pixels_[y_ * width() + x_] = color;
    ++pixel_writes_;
  }
  if (++x_ > x1_) { x_ = x0_; if (++y_ > y1_) y_ = y0_; }
}

Bmp3xx::Bmp3xx() {
  registers_[0] = 0x50;  // BMP388; caller can select BMP390 (0x60)
  registers_[3] = 0x70;  // command ready, temperature and pressure ready
  // Synthetic nonzero calibration fixture. This is not a per-device factory
  // calibration capture and makes no claim of electrical sensor accuracy.
  const uint8_t calibration[21] = {
    0x70,0x6b,0x43,0x67,0x03,0x67,0x46,0x50,0x4a,0x0c,0x00,
    0xf4,0x35,0x8b,0x0c,0x00,0x00,0x10,0x00,0x00,0x00};
  std::copy_n(calibration, 21, registers_.begin() + 0x31);
  uint8_t crc = 0xff;
  for (uint8_t byte : calibration) {
    for (unsigned bit = 0; bit < 8; ++bit) {
      const bool carry = (crc ^ byte) & 0x80;
      crc <<= 1; byte <<= 1;
      if (carry) crc ^= 0x1d;
    }
  }
  registers_[0x30] = crc ^ 0xff;
  raw_sample(8000000, 8000000);
}
bool Bmp3xx::read(uint8_t address, uint8_t* data, size_t length) const {
  if (!connected_ || !data || length > registers_.size() - address) return false;
  std::copy_n(registers_.begin() + address, length, data);
  return true;
}
bool Bmp3xx::write(uint8_t address, const uint8_t* data, size_t length) {
  if (!connected_ || !data || length > registers_.size() - address) return false;
  std::copy_n(data, length, registers_.begin() + address);
  if (address == 0x7e && length && data[0] == 0xb6) {
    // Soft reset retains OTP calibration and chip ID.
    registers_[0x1b] = registers_[0x1c] = registers_[0x1d] = registers_[0x1f] = 0;
    registers_[3] = 0x70;
  }
  return true;
}
void Bmp3xx::raw_sample(uint32_t pressure, uint32_t temperature) {
  for (unsigned i = 0; i < 3; ++i) {
    registers_[4+i] = uint8_t(pressure >> (i*8));
    registers_[7+i] = uint8_t(temperature >> (i*8));
  }
}

NorFlash::NorFlash(size_t size) : bytes_(new uint8_t[size]), size_(size) {
  std::fill_n(bytes_, size_, 0xff);
}
NorFlash::~NorFlash() { delete[] bytes_; }
bool NorFlash::read(size_t offset, uint8_t* data, size_t length) const {
  if (!data || offset > size_ || length > size_ - offset) return false;
  std::copy_n(bytes_ + offset, length, data);
  return true;
}
bool NorFlash::program(size_t offset, const uint8_t* data, size_t length) {
  if (!data || offset > size_ || length > size_ - offset) return false;
  for (size_t i = 0; i < length; ++i) {
    if ((bytes_[offset+i] & data[i]) != data[i] || write_budget_ == 0) return false;
    bytes_[offset+i] &= data[i];
    if (write_budget_ != static_cast<size_t>(-1)) --write_budget_;
  }
  return true;
}
bool NorFlash::erase(size_t offset, size_t length) {
  if ((offset % 4096) || (length % 4096) || offset > size_ || length > size_ - offset) return false;
  for (size_t i = 0; i < length; ++i) {
    if (!write_budget_) return false;
    bytes_[offset+i] = 0xff;
    if (write_budget_ != static_cast<size_t>(-1)) --write_budget_;
  }
  return true;
}

}  // namespace eppg::sim::devices
