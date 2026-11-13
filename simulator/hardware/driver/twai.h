#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int esp_err_t;
typedef uint32_t TickType_t;

#define ESP_OK 0
#define ESP_FAIL (-1)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define TWAI_FRAME_MAX_DLC 8

#define TWAI_ALERT_RX_DATA (1U << 0)
#define TWAI_ALERT_ERR_PASS (1U << 1)
#define TWAI_ALERT_BUS_ERROR (1U << 2)
#define TWAI_ALERT_RX_QUEUE_FULL (1U << 3)

typedef struct {
  uint32_t identifier;
  uint8_t data_length_code;
  uint8_t data[8];
  uint8_t extd;
  uint8_t rtr;
} twai_message_t;

typedef struct {
  uint32_t bus_error_count;
  uint32_t msgs_to_rx;
  uint32_t rx_missed_count;
  uint32_t rx_overrun_count;
} twai_status_info_t;

esp_err_t twai_read_alerts(uint32_t* alerts, TickType_t ticks_to_wait);
esp_err_t twai_get_status_info(twai_status_info_t* status);
esp_err_t twai_receive(twai_message_t* message, TickType_t ticks_to_wait);
esp_err_t twai_transmit(const twai_message_t* message,
                        TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
