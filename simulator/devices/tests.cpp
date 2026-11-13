#include "peripherals.h"

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace eppg::sim::devices;
static void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
static void mcp_write(Mcp2515& mcp, uint8_t reg, uint8_t value) {
  const uint8_t bytes[] = {2, reg, value};
  mcp.transfer(bytes, nullptr, sizeof(bytes));
}
static void test_mcp_registers_and_buffers() {
  Mcp2515 mcp;
  check(mcp.reg(0x0f) == 0x80, "MCP reset did not enter configuration mode");
  mcp_write(mcp, 0x0f, 0);
  check(mcp.reg(0x0e) == 0, "MCP CANSTAT did not follow requested mode");
  mcp_write(mcp, 0x2a, 0x41); mcp_write(mcp, 0x29, 0xf1); mcp_write(mcp, 0x28, 0x85);
  const uint8_t read_timing[] = {3, 0x28, 0, 0, 0};
  uint8_t result[16]{};
  mcp.transfer(read_timing, result, sizeof(read_timing));
  check(result[2] == 0x85 && result[3] == 0xf1 && result[4] == 0x41,
        "MCP multi-register reads changed byte order");
  CanFrame frame; frame.id = 0x18ff28f4; frame.length = 8;
  frame.data = {0x18,90,0x88,0x13,0xd2,3,0,0};
  check(mcp.receive(frame) && mcp.receive(frame), "MCP did not fill two receive buffers");
  check(!mcp.receive(frame) && mcp.overruns() == 1, "MCP overrun was hidden by unbounded buffers");
  const uint8_t read_buffer[14] = {0x90};
  mcp.transfer(read_buffer, result, sizeof(read_buffer));
  check(result[1] == uint8_t(frame.id >> 21) && result[6] == frame.data[0],
        "MCP RX identifier/data register encoding incorrect");
  check((mcp.reg(0x2c) & 3) == 2, "READ_RX_BUFFER did not clear only the selected flag");
  check(mcp.receive(frame), "MCP freed buffer did not accept next frame");
  const uint8_t modify[] = {5,0x2c,3,0};
  mcp.transfer(modify, nullptr, sizeof(modify));
  check((mcp.reg(0x2c) & 3) == 0, "MCP bit modify did not preserve mask semantics");
}
static void test_mcp_transmit_loopback() {
  Mcp2515 mcp;
  mcp_write(mcp,0x0f,0x40);
  const uint8_t packet[] = {0x40,0x24,0x60,0,0,2,0xaa,0x55};
  mcp.transfer(packet,nullptr,sizeof(packet));
  const uint8_t rts[] = {0x81};
  mcp.transfer(rts,nullptr,sizeof(rts));
  CanFrame sent;
  check(mcp.take_transmitted(&sent) && sent.id == 0x123 && !sent.extended &&
        sent.length == 2 && sent.data[0] == 0xaa && sent.data[1] == 0x55,
        "MCP loaded transmit buffer did not become correct CAN frame");
  check((mcp.reg(0x2c) & 5) == 5, "MCP loopback did not set RX and TX completion flags");
}
static void lcd_command(St7735& lcd,uint8_t command, const std::array<uint8_t,4>& data) {
  lcd.transfer(false,&command,1); lcd.transfer(true,data.data(),data.size());
}
static void test_display_spi_stream() {
  St7735 lcd;
  const uint8_t madctl=0x36, landscape=0x60, on=0x29;
  lcd.transfer(false,&madctl,1);lcd.transfer(true,&landscape,1);
  check(lcd.width()==160 && lcd.height()==128,"ST7735 MADCTL dimensions wrong");
  lcd.transfer(false,&on,1);
  lcd_command(lcd,0x2a,{0,2,0,3});lcd_command(lcd,0x2b,{0,1,0,2});
  const uint8_t ramwr=0x2c;
  lcd.transfer(false,&ramwr,1);
  const uint8_t pixels[]={0xf8,0,0x07,0xe0,0,0x1f,0xff,0xff};
  // Deliberately split a pixel across transfers, as DMA chunks may do.
  lcd.transfer(true,pixels,3);lcd.transfer(true,pixels+3,5);
  check(lcd.on()&&lcd.pixel(2,1)==0xf800&&lcd.pixel(3,1)==0x07e0&&
        lcd.pixel(2,2)==0x001f&&lcd.pixel(3,2)==0xffff,
        "ST7735 SPI command stream did not produce RGB565 framebuffer");
  check(lcd.pixel_writes()==4&&lcd.invalid_windows()==0,"ST7735 pixel accounting wrong");
  lcd_command(lcd,0x2a,{0,159,0,160});lcd.transfer(false,&ramwr,1);
  check(lcd.invalid_windows()==1,"ST7735 accepted off-panel address window silently");
}
static void test_bmp_register_fixture() {
  Bmp3xx bmp;
  uint8_t bytes[6]{};
  check(bmp.read(0,bytes,1)&&bytes[0]==0x50,"BMP chip ID missing");
  std::array<uint8_t, 21> trim{};
  check(bmp.read(0x31,trim.data(),trim.size()),"BMP calibration fixture missing");
  uint8_t crc = 0xff;
  for (uint8_t byte : trim) for (unsigned bit=0;bit<8;++bit) {
    const bool carry=(crc^byte)&0x80;crc<<=1;byte<<=1;if(carry)crc^=0x1d;
  }
  check(bmp.read(0x30,bytes,1)&&bytes[0]==uint8_t(crc^0xff),
        "BMP OTP calibration CRC does not match production validation");
  bmp.raw_sample(0x123456,0xabcdef);
  check(bmp.read(4,bytes,6)&&bytes[0]==0x56&&bytes[2]==0x12&&
        bytes[3]==0xef&&bytes[5]==0xab,"BMP raw ADC byte order wrong");
  const uint8_t configuration=0x15;
  check(bmp.write(0x1c,&configuration,1)&&bmp.read(0x1c,bytes,1)&&bytes[0]==0x15,
        "BMP oversampling register did not retain writes");
  const uint8_t reset=0xb6;
  check(bmp.write(0x7e,&reset,1)&&bmp.read(0x1c,bytes,1)&&bytes[0]==0,
        "BMP soft reset did not clear configuration");
  bmp.set_connected(false);
  check(!bmp.read(0,bytes,1),"BMP disconnected device still answered I2C");
}
static void test_flash_power_cut() {
  NorFlash flash(8192);
  const uint8_t image[]={0x11,0x22,0x33,0x44};
  uint8_t bytes[4]{};
  flash.cut_power_after(2);
  check(!flash.program(4096,image,4),"NOR program ignored injected power cut");
  check(flash.read(4096,bytes,4)&&bytes[0]==0x11&&bytes[1]==0x22&&
        bytes[2]==0xff&&bytes[3]==0xff,"NOR partial program not retained after power loss");
  flash.restore_power();
  const uint8_t erased[]={0xff};
  check(!flash.program(4096,erased,1),"NOR allowed programmed zero bits to become ones");
  check(!flash.erase(4097,4096),"NOR accepted unaligned erase");
  check(flash.erase(4096,4096)&&flash.program(4096,image,4),"NOR erase/program recovery failed");
  check(!flash.read(8191,bytes,4),"NOR out of bounds read accepted");
}
int main() {
  try {
    test_mcp_registers_and_buffers();test_mcp_transmit_loopback();
    test_display_spi_stream();test_bmp_register_fixture();test_flash_power_cut();
    std::cout<<"5 register-device integration tests passed\n";
    return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
