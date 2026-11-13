#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t eppg_millis(void);
uint64_t eppg_micros(void);
void eppg_delay_ms(uint32_t ms);
int eppg_gpio_config(int pin, bool output, bool pullup);
int eppg_gpio_read(int pin);
void eppg_gpio_write(int pin, int value);
int eppg_adc_read(int pin);
int eppg_pwm_config(int channel, int pin, uint32_t hz);
void eppg_pwm_write(int channel, uint32_t duty);
void eppg_pwm_frequency(int channel, uint32_t hz);
int eppg_pixel_write(int pin, uint32_t rgb);
int eppg_spi_bus_init(int mosi, int miso, int sclk);
void* eppg_spi_add_device(int cs, uint32_t hz, uint8_t mode);
int eppg_spi_transfer(void* device, const uint8_t* tx, uint8_t* rx, size_t length);
int eppg_i2c_init(int sda, int scl);
int eppg_i2c_read(uint8_t address, uint8_t reg, uint8_t* data, size_t length);
int eppg_i2c_write(uint8_t address, uint8_t reg, const uint8_t* data, size_t length);
int eppg_usb_init(void);
int eppg_usb_read(uint8_t* data, size_t length);
int eppg_usb_write(const uint8_t* data, size_t length);
float eppg_cpu_temperature(void);

#ifdef __cplusplus
}
#endif
