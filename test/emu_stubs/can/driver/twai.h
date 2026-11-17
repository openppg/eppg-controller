// Declarations of the ESP-IDF 4.4.7 legacy TWAI driver API (the version
// espressif32@6.13.0 pins for arduino+espidf builds) as used by esc.cpp and the
// SINE-ESC-CAN CanardAdapter. Each test that includes this provides the
// definitions, modelling the driver's behaviour.
#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_NOT_SUPPORTED 0x106
inline const char* esp_err_to_name(esp_err_t) { return "esp_err"; }

typedef int gpio_num_t;
#define TWAI_IO_UNUSED (-1)
#define TWAI_FRAME_MAX_DLC 8

#define TWAI_ALERT_RX_DATA 0x00000004
#define TWAI_ALERT_BUS_ERROR 0x00000200
#define TWAI_ALERT_RX_QUEUE_FULL 0x00000800
#define TWAI_ALERT_ERR_PASS 0x00001000

typedef enum {
  TWAI_MODE_NORMAL,
  TWAI_MODE_NO_ACK,
  TWAI_MODE_LISTEN_ONLY,
} twai_mode_t;

typedef enum {
  TWAI_STATE_STOPPED,
  TWAI_STATE_RUNNING,
  TWAI_STATE_BUS_OFF,
  TWAI_STATE_RECOVERING,
} twai_state_t;

typedef struct {
  union {
    struct {
      uint32_t extd : 1;
      uint32_t rtr : 1;
      uint32_t ss : 1;
      uint32_t self : 1;
      uint32_t dlc_non_comp : 1;
      uint32_t reserved : 27;
    };
    uint32_t flags;
  };
  uint32_t identifier;
  uint8_t data_length_code;
  uint8_t data[TWAI_FRAME_MAX_DLC];
} twai_message_t;

typedef struct {
  twai_state_t state;
  uint32_t msgs_to_tx;
  uint32_t msgs_to_rx;
  uint32_t tx_error_counter;
  uint32_t rx_error_counter;
  uint32_t tx_failed_count;
  uint32_t rx_missed_count;
  uint32_t rx_overrun_count;
  uint32_t arb_lost_count;
  uint32_t bus_error_count;
} twai_status_info_t;

typedef struct {
  twai_mode_t mode;
  gpio_num_t tx_io;
  gpio_num_t rx_io;
  gpio_num_t clkout_io;
  gpio_num_t bus_off_io;
  uint32_t tx_queue_len;
  uint32_t rx_queue_len;
  uint32_t alerts_enabled;
  uint32_t clkout_divider;
  int intr_flags;
} twai_general_config_t;

typedef struct {
  uint32_t brp;
  uint8_t tseg_1;
  uint8_t tseg_2;
  uint8_t sjw;
  bool triple_sampling;
} twai_timing_config_t;

typedef struct {
  uint32_t acceptance_code;
  uint32_t acceptance_mask;
  bool single_filter;
} twai_filter_config_t;

#define TWAI_GENERAL_CONFIG_DEFAULT(tx_io_num, rx_io_num, op_mode)                       \
  {.mode = op_mode, .tx_io = tx_io_num, .rx_io = rx_io_num, .clkout_io = TWAI_IO_UNUSED, \
   .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 5, .rx_queue_len = 5,                  \
   .alerts_enabled = 0, .clkout_divider = 0, .intr_flags = 0}
#define TWAI_TIMING_CONFIG_1MBITS() {.brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true}

esp_err_t twai_driver_install(const twai_general_config_t* g_config, const twai_timing_config_t* t_config,
                              const twai_filter_config_t* f_config);
esp_err_t twai_driver_uninstall(void);
esp_err_t twai_start(void);
esp_err_t twai_stop(void);
esp_err_t twai_transmit(const twai_message_t* message, TickType_t ticks_to_wait);
esp_err_t twai_receive(twai_message_t* message, TickType_t ticks_to_wait);
esp_err_t twai_read_alerts(uint32_t* alerts, TickType_t ticks_to_wait);
esp_err_t twai_reconfigure_alerts(uint32_t alerts_enabled, uint32_t* current_alerts);
esp_err_t twai_initiate_recovery(void);
esp_err_t twai_get_status_info(twai_status_info_t* status_info);
esp_err_t twai_clear_transmit_queue(void);
