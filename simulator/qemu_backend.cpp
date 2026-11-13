// Application-level peripheral models for Espressif ESP32-S3 QEMU.
// CPU, interrupts, FreeRTOS and flash execute in QEMU. Unsupported SPI/I2C/
// GPIO/ADC devices are intercepted at the native platform driver boundary.
#include "devices/peripherals.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "cJSON.h"
#include "sp140/device_state.h"
#include "esp_heap_caps.h"
#include "platform/staging.h"

#ifdef EPPG_RUST
extern "C" uint32_t eppg_qemu_state();
extern "C" uint32_t eppg_qemu_last_throttle();
extern "C" void* eppg_qemu_task_handle(int kind);
#else
extern TaskHandle_t throttleTaskHandle;
extern TaskHandle_t uiTaskHandle;
extern TaskHandle_t bmsTaskHandle;
extern volatile uint32_t lastThrottleRunMs;
extern volatile DeviceState currentState;
#endif

namespace {
using namespace eppg::sim::devices;
uint32_t application_state() {
#ifdef EPPG_RUST
  return eppg_qemu_state();
#else
  return static_cast<uint32_t>(currentState);
#endif
}
uint32_t application_throttle_time() {
#ifdef EPPG_RUST
  return eppg_qemu_last_throttle();
#else
  return lastThrottleRunMs;
#endif
}
TaskHandle_t application_task(int kind) {
#ifdef EPPG_RUST
  return static_cast<TaskHandle_t>(eppg_qemu_task_handle(kind));
#else
  return kind == 0 ? throttleTaskHandle : kind == 1 ? uiTaskHandle : bmsTaskHandle;
#endif
}
unsigned free_stack(int kind) {
  const TaskHandle_t handle = application_task(kind);
  return handle ? static_cast<unsigned>(uxTaskGetStackHighWaterMark(handle)) : 0;
}
St7735 display;
Mcp2515 mcp;
Bmp3xx barometer;
std::array<int, 49> gpio{};
SemaphoreHandle_t mutex = nullptr;
uint32_t bms_sequence = 0;
uint32_t previous_bms_poll = 0;
uint32_t previous_report = 0;
bool automatic_inputs = true;
bool boot_complete = false;
uint32_t control_started_ms = 0;
uint16_t manual_throttle = 0;
bool manual_button = false;
bool bms_link = true;
bool esc_fault = false;
uint32_t pixel_rgb = 0;
std::array<uint32_t, 4> pwm_frequency{};
std::array<uint32_t, 4> pwm_duty{};
QueueHandle_t console_input = nullptr;
void set_can_fault(bool active);
uint32_t now_ms() { return uint32_t(esp_timer_get_time() / 1000); }

class Lock {
 public:
  Lock() {
    // Peripheral initialization is single-threaded before application tasks.
    if (!mutex) mutex = xSemaphoreCreateMutex();
    if (mutex) xSemaphoreTake(mutex, portMAX_DELAY);
  }
  ~Lock() { if (mutex) xSemaphoreGive(mutex); }
};
CanFrame battery_frame(unsigned sequence) {
  CanFrame f; f.length = 8;
  switch (sequence % 9) {
    case 0: f.id = 0x18ff28f4; f.data = {0x18,90,0x88,0x13,0xd2,3,0,0}; break;
    case 1: f.id = 0x18fe28f4; f.data = {0x08,0x10,0xf4,0x0f,65,65,0,0}; break;
    case 2: f.id = 0x18b428f4; f.data = {65,65,65,65,65,65,0,0}; break;
    default:
      f.id = 0x18c828f4 + ((sequence % 9 - 3) << 16);
      f.data = {0,0x10,0,0x10,0,0x10,0,0x10};
      break;
  }
  return f;
}
void report() {
  const uint32_t now = now_ms();
  if (now - previous_report < 1000) return;
  previous_report = now;
  // Stable evidence marker consumed by qemu_smoke.py. Screen checksum is of
  // bytes actually written by production display/SPI code, not a fake screen.
  uint32_t hash = 2166136261u;
  for (uint16_t pixel : display.pixels()) {
    hash = (hash ^ uint8_t(pixel)) * 16777619u;
    hash = (hash ^ uint8_t(pixel >> 8)) * 16777619u;
  }
  std::printf("EPPG_QEMU {\"time_ms\":%lu,\"pixels\":%llu,\"screen_fnv1a\":%lu,\"mcp_overruns\":%llu,"
      "\"state\":%u,\"throttle_age_ms\":%lu,\"throttle_stack_free\":%u,\"ui_stack_free\":%u,"
      "\"bms_stack_free\":%u,\"largest_heap\":%u,"
      "\"buzzer_hz\":%lu,\"buzzer_duty\":%lu,\"vibe_duty\":%lu,\"pixel_rgb\":%lu}\n",
      static_cast<unsigned long>(now), static_cast<unsigned long long>(display.pixel_writes()),
      static_cast<unsigned long>(hash), static_cast<unsigned long long>(mcp.overruns()),
      static_cast<unsigned>(application_state()), static_cast<unsigned long>(now-application_throttle_time()),
      free_stack(0), free_stack(1), free_stack(2),
      static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
      static_cast<unsigned long>(pwm_frequency[1]), static_cast<unsigned long>(pwm_duty[1]),
      static_cast<unsigned long>(pwm_duty[0]), static_cast<unsigned long>(pixel_rgb));
}
}

extern "C" int eppg_qemu_gpio_read(int pin) {
  Lock lock;
  if (pin < 0 || pin >= int(gpio.size())) return 0;
  // Automatic smoke scenario exercises the real button debounce/gesture task.
  // First click 4.0-4.1s, second hold 4.4-6.7s, after the 2s boot splash.
  if (pin == 1) {
    if (!automatic_inputs) return manual_button ? 0 : 1;
    if (!boot_complete) return 1;
    const uint32_t elapsed = now_ms() - control_started_ms;
    return ((elapsed >= 200 && elapsed < 300) || (elapsed >= 600 && elapsed < 2900)) ? 0 : 1;
  }
  return gpio[size_t(pin)];
}
extern "C" void eppg_qemu_gpio_write(int pin, int value) {
  Lock lock;
  if (pin >= 0 && pin < int(gpio.size())) gpio[size_t(pin)] = value != 0;
}
extern "C" int eppg_qemu_adc_read(int) {
  Lock lock;
  if (!automatic_inputs) return manual_throttle;
  if (!boot_complete) return 0;
  const uint32_t now = now_ms() - control_started_ms;
  // Raw 12-bit input stays released until the arm gesture completes.
  return now < 3400 ? 0 : int(std::min<uint32_t>(4095, (now - 3400) * 2));
}
extern "C" int eppg_qemu_spi_transfer(void* handle, const uint8_t* tx, uint8_t* rx, size_t length) {
  Lock lock;
  const int cs = int(reinterpret_cast<uintptr_t>(handle)) - 1;
  if (rx) std::fill_n(rx, length, 0);
  if (cs == 10) {
    display.transfer(gpio[14] != 0, tx, length);
  } else if (cs == 5) {
    const uint32_t now = now_ms();
    if (bms_link && now - previous_bms_poll >= 20) {
      previous_bms_poll = now;
      mcp.receive(battery_frame(bms_sequence++));
    }
    mcp.transfer(tx, rx, length);
  } else return -1;
  report();
  return 0;
}
extern "C" int eppg_qemu_i2c_read(uint8_t address, uint8_t reg, uint8_t* data, size_t length) {
  Lock lock;
  if (address != 0x77 && address != 0x76) return -1;
  return barometer.read(reg, data, length) ? 0 : -1;
}
extern "C" int eppg_qemu_i2c_write(uint8_t address, uint8_t reg, const uint8_t* data, size_t length) {
  Lock lock;
  if (address != 0x77 && address != 0x76) return -1;
  return barometer.write(reg, data, length) ? 0 : -1;
}

extern "C" void eppg_qemu_boot_complete() {
  Lock lock;
  boot_complete = true;
  control_started_ms = now_ms();
}
extern "C" void eppg_qemu_pwm_config(int channel, int, uint32_t hz) {
  Lock lock;
  if (channel >= 0 && channel < int(pwm_frequency.size())) pwm_frequency[channel] = hz;
}
extern "C" void eppg_qemu_pwm_frequency(int channel, uint32_t hz) {
  Lock lock;
  if (channel >= 0 && channel < int(pwm_frequency.size())) pwm_frequency[channel] = hz;
}
extern "C" void eppg_qemu_pwm_write(int channel, uint32_t duty) {
  Lock lock;
  if (channel >= 0 && channel < int(pwm_duty.size())) pwm_duty[channel] = std::min<uint32_t>(255, duty);
}
extern "C" void eppg_qemu_pixel_write(int, uint32_t rgb) {
  Lock lock;
  pixel_rgb = rgb;
}

namespace {
void staging_self_test() {
  constexpr uint32_t size = 256u * 1024u;
  uint8_t bytes[512], actual[512];
  bool ok = application_state() == 0 && eppg_stage_begin(size) == 0;
  for (uint32_t offset = 0; ok && offset < size; offset += sizeof(bytes)) {
    for (size_t i = 0; i < sizeof(bytes); ++i) bytes[i] = static_cast<uint8_t>((offset + i) * 37u + ((offset + i) >> 8));
    ok = eppg_stage_write(offset, bytes, sizeof(bytes)) == static_cast<int32_t>(offset + sizeof(bytes));
  }
  ok = ok && eppg_stage_finish() == 0;
  for (uint32_t offset = 0; ok && offset < size; offset += sizeof(actual)) {
    ok = eppg_stage_read(offset, actual, sizeof(actual)) == 0;
    for (size_t i = 0; ok && i < sizeof(actual); ++i)
      ok = actual[i] == static_cast<uint8_t>((offset + i) * 37u + ((offset + i) >> 8));
  }
  eppg_stage_abort();
  std::printf("EPPG_QEMU_STAGE size=%lu verified=%u scratch=4096\n", static_cast<unsigned long>(size), ok ? 1 : 0);
}
bool emulator_command(const char* text) {
  cJSON* document = cJSON_Parse(text);
  if (!document) return false;
  const cJSON* command = cJSON_GetObjectItemCaseSensitive(document, "emulator");
  if (!cJSON_IsObject(command)) { cJSON_Delete(document); return false; }
  bool stage_test = false;
  {
    Lock lock;
    const cJSON* value = cJSON_GetObjectItemCaseSensitive(command, "auto");
    if (cJSON_IsBool(value)) automatic_inputs = cJSON_IsTrue(value);
    value = cJSON_GetObjectItemCaseSensitive(command, "throttle");
    if (cJSON_IsNumber(value) && value->valuedouble >= 0 && value->valuedouble <= 4095) {
      manual_throttle = value->valuedouble;
      automatic_inputs = false;
    }
    value = cJSON_GetObjectItemCaseSensitive(command, "button");
    if (cJSON_IsBool(value)) { manual_button = cJSON_IsTrue(value); automatic_inputs = false; }
    value = cJSON_GetObjectItemCaseSensitive(command, "bms_link");
    if (cJSON_IsBool(value)) bms_link = cJSON_IsTrue(value);
    value = cJSON_GetObjectItemCaseSensitive(command, "barometer_link");
    if (cJSON_IsBool(value)) barometer.set_connected(cJSON_IsTrue(value));
    value = cJSON_GetObjectItemCaseSensitive(command, "esc_bus_off");
    if (cJSON_IsBool(value)) set_can_fault(cJSON_IsTrue(value));
    value = cJSON_GetObjectItemCaseSensitive(command, "stage_test");
    if (cJSON_IsTrue(value) && application_state() == 0) {
      stage_test = true;
      automatic_inputs = false;
      manual_throttle = 0;
      manual_button = false;
    }
  }
  cJSON_Delete(document);
  std::printf("EPPG_QEMU_INPUT accepted\n");
  if (stage_test) staging_self_test();
  return true;
}
void console_task(void*) {
  char line[1024];
  size_t used = 0;
  bool overflow = false;
  for (;;) {
    uint8_t byte = 0;
    if (uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(50)) != 1) continue;
    if (byte == '\n') {
      line[used] = '\0';
      if (!overflow && !emulator_command(line)) {
        for (size_t i = 0; i < used; ++i) xQueueSend(console_input, &line[i], 0);
        xQueueSend(console_input, &byte, 0);
      }
      used = 0;
      overflow = false;
    } else if (used + 1 < sizeof(line)) line[used++] = byte;
    else overflow = true;
  }
}
}
extern "C" void eppg_qemu_console_start() {
  console_input = xQueueCreate(2048, 1);
  configASSERT(console_input);
  configASSERT(xTaskCreate(console_task, "QemuConsole", 6144, nullptr, 1, nullptr) == pdPASS);
}
extern "C" int eppg_qemu_usb_read(uint8_t* data, size_t length) {
  size_t count = 0;
  while (console_input && count < length && xQueueReceive(console_input, data + count, 0) == pdTRUE) ++count;
  return count;
}

// The S3 machine includes TWAI emulation, but a real ESC is external hardware.
// Linker wrappers attach an in-process ESC node at the CAN frame boundary so
// production libcanard serialization, CRC, fragmentation and reassembly still
// run on the emulated Xtensa CPU. This models the wire endpoint, not RF.
#include "driver/twai.h"
#include "canard.h"
#include "sine_esc.SetThrottleSettings2_req.h"
#include "sine_esc.SetThrottleSettings2_res.h"
#include "sine_esc.GetHwInfo_res.h"

namespace {
alignas(8) std::array<uint8_t, 4096> esc_pool{};
CanardInstance esc_node{};
struct ScheduledCan { CanardCANFrame frame{}; uint64_t due_us = 0; };
std::array<ScheduledCan, 64> can_rx{};
size_t can_rx_head = 0, can_rx_count = 0;
bool can_installed = false;
twai_state_t can_state = TWAI_STATE_STOPPED;
uint32_t can_alert_mask = 0;
uint32_t can_pending_alerts = 0;
uint32_t can_received = 0;
uint32_t can_transmitted = 0;
uint32_t can_dropped = 0;
uint64_t can_available_us = 0;
uint16_t esc_pwm_01us = 10000;

void set_can_fault(bool active) {
  esc_fault = active;
  if (active) {
    can_state = TWAI_STATE_BUS_OFF;
    can_rx_count = 0;
    can_pending_alerts |= TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS;
  } else if (can_state == TWAI_STATE_RECOVERING) {
    can_state = TWAI_STATE_STOPPED;
    can_pending_alerts |= TWAI_ALERT_BUS_RECOVERED;
  }
}

bool accept_esc(const CanardInstance*, uint64_t* signature, uint16_t id,
                CanardTransferType type, uint8_t) {
  if (type != CanardTransferTypeRequest) return false;
  if (id == SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_ID) {
    *signature = SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_SIGNATURE; return true;
  }
  if (id == SINE_ESC_GETHWINFO_RESPONSE_ID) {
    *signature = SINE_ESC_GETHWINFO_RESPONSE_SIGNATURE; return true;
  }
  return false;
}
void esc_response(const CanardRxTransfer* request, uint64_t signature,
                  const uint8_t* payload, uint16_t length) {
  uint8_t transfer_id = request->transfer_id;
  CanardTxTransfer tx{};
  tx.transfer_type = CanardTransferTypeResponse;
  tx.data_type_signature = signature;
  tx.data_type_id = request->data_type_id;
  tx.inout_transfer_id = &transfer_id;
  tx.priority = CANARD_TRANSFER_PRIORITY_HIGHEST;
  tx.payload = payload; tx.payload_len = length;
  canardRequestOrRespondObj(&esc_node, request->source_node_id, &tx);
}
void receive_esc(CanardInstance*, CanardRxTransfer* transfer) {
  if (transfer->data_type_id == SINE_ESC_SETTHROTTLESETTINGS2_REQUEST_ID) {
    sine_esc_SetThrottleSettings2Request request{};
    if (sine_esc_SetThrottleSettings2Request_decode(transfer, &request)) return;
    esc_pwm_01us = request.pwm_us;
    sine_esc_SetThrottleSettings2Response reply{};
    reply.recv_pwm = reply.comm_pwm = esc_pwm_01us;
    const float power = std::max(0.0f, std::min(1.0f, (int(esc_pwm_01us) - 10000) / 9500.0f));
    reply.speed = int32_t(power * 12000);
    reply.bus_current = int16_t(power * power * 1200);
    reply.current = int16_t(reply.bus_current * 1.7f);
    reply.voltage = uint16_t(1008 - reply.bus_current * 0.012f);
    reply.v_modulation = uint16_t(power * 1000);
    reply.mos_temp = int16_t(250 + power * 500);
    reply.cap_temp = reply.mos_temp - 30; reply.mcu_temp = reply.mos_temp - 70;
    reply.motor_temp = reply.mos_temp + 50;
    reply.time_10ms = uint16_t(now_ms() / 10);
    uint8_t payload[SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_MAX_SIZE]{};
    const auto size = sine_esc_SetThrottleSettings2Response_encode(&reply, payload);
    esc_response(transfer, SINE_ESC_SETTHROTTLESETTINGS2_RESPONSE_SIGNATURE, payload, uint16_t(size));
  } else if (transfer->data_type_id == SINE_ESC_GETHWINFO_RESPONSE_ID) {
    sine_esc_GetHwInfoResponse reply{};
    reply.hardware_id = 1; reply.app_version = 801; reply.bootloader_version = 1;
    const uint8_t identity[16] = {'Q','E','M','U','-','E','S','C','-','0','0','0','0','0','0','1'};
    std::copy_n(identity, sizeof(identity), reply.sn_code);
    uint8_t payload[SINE_ESC_GETHWINFO_RESPONSE_MAX_SIZE]{};
    const auto size = sine_esc_GetHwInfoResponse_encode(&reply, payload);
    esc_response(transfer, SINE_ESC_GETHWINFO_RESPONSE_SIGNATURE, payload, uint16_t(size));
  }
}
void flush_esc_frames() {
  while (const CanardCANFrame* frame = canardPeekTxQueue(&esc_node)) {
    if (can_rx_count == can_rx.size()) { ++can_dropped; can_pending_alerts |= TWAI_ALERT_RX_QUEUE_FULL; }
    else {
      auto& scheduled = can_rx[(can_rx_head + can_rx_count) % can_rx.size()];
      scheduled.frame = *frame;
      // Extended classic-CAN serialization at 1 Mbps, with stuffing allowance.
      const uint64_t bits = 67 + 8 * frame->data_len + (34 + 8 * frame->data_len) / 4;
      can_available_us = std::max<uint64_t>(can_available_us, esp_timer_get_time()) + bits;
      scheduled.due_us = can_available_us;
      ++can_rx_count;
    }
    canardPopTxQueue(&esc_node);
  }
}
size_t ready_can_frames() {
  size_t n = 0;
  while (n < can_rx_count && can_rx[(can_rx_head+n) % can_rx.size()].due_us <= uint64_t(esp_timer_get_time())) ++n;
  return n;
}
}

extern "C" esp_err_t __wrap_twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*, const twai_filter_config_t*) {
  Lock lock;
  if (can_installed) return ESP_ERR_INVALID_STATE;
  canardInit(&esc_node, esc_pool.data(), esc_pool.size(), receive_esc, accept_esc, nullptr);
  canardSetLocalNodeID(&esc_node, 0x20);
  can_installed = true; can_state = TWAI_STATE_STOPPED;
  can_rx_head = can_rx_count = 0;
  std::printf("EPPG_QEMU_CAN node=32 bitrate=1000000 backend=libcanard-frame-peer\n");
  return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_driver_uninstall() {
  Lock lock; can_installed = false; can_rx_count = 0; return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_start() {
  Lock lock;
  if (!can_installed) return ESP_ERR_INVALID_STATE;
  can_state = TWAI_STATE_RUNNING; return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_stop() {
  Lock lock;
  if (!can_installed) return ESP_ERR_INVALID_STATE;
  can_state = TWAI_STATE_STOPPED; return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_transmit(const twai_message_t* message, TickType_t) {
  Lock lock;
  if (!can_installed || can_state != TWAI_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
  if (!message || message->data_length_code > 8) return ESP_ERR_INVALID_ARG;
  CanardCANFrame frame{};
  frame.id = message->identifier | (message->extd ? CANARD_CAN_FRAME_EFF : 0) |
             (message->rtr ? CANARD_CAN_FRAME_RTR : 0);
  frame.data_len = message->data_length_code;
  std::copy_n(message->data, frame.data_len, frame.data);
  ++can_transmitted;
  canardHandleRxFrame(&esc_node, &frame, esp_timer_get_time());
  flush_esc_frames();
  canardCleanupStaleTransfers(&esc_node, esp_timer_get_time());
  return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_receive(twai_message_t* message, TickType_t) {
  Lock lock;
  if (!can_installed) return ESP_ERR_INVALID_STATE;
  if (!message) return ESP_ERR_INVALID_ARG;
  if (!ready_can_frames()) return ESP_ERR_TIMEOUT;
  const auto frame = can_rx[can_rx_head].frame;
  can_rx_head = (can_rx_head + 1) % can_rx.size(); --can_rx_count;
  *message = {};
  message->identifier = frame.id & CANARD_CAN_EXT_ID_MASK;
  message->extd = (frame.id & CANARD_CAN_FRAME_EFF) != 0;
  message->rtr = (frame.id & CANARD_CAN_FRAME_RTR) != 0;
  message->data_length_code = frame.data_len;
  std::copy_n(frame.data, frame.data_len, message->data);
  if (++can_received % 250 == 0) {
    std::printf("EPPG_QEMU_CAN tx=%lu rx=%lu pwm_01us=%u dropped=%lu\n",
        static_cast<unsigned long>(can_transmitted), static_cast<unsigned long>(can_received),
        esc_pwm_01us, static_cast<unsigned long>(can_dropped));
  }
  return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_get_status_info(twai_status_info_t* status) {
  Lock lock;
  if (!can_installed) return ESP_ERR_INVALID_STATE;
  if (!status) return ESP_ERR_INVALID_ARG;
  *status = {}; status->state = can_state;
  status->msgs_to_rx = ready_can_frames();
  status->rx_missed_count = can_dropped;
  status->tx_error_counter = esc_fault ? 256 : 0;
  return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_read_alerts(uint32_t* alerts, TickType_t) {
  Lock lock;
  if (!can_installed) return ESP_ERR_INVALID_STATE;
  if (!alerts) return ESP_ERR_INVALID_ARG;
  *alerts = (can_pending_alerts | (ready_can_frames() ? TWAI_ALERT_RX_DATA : 0)) & can_alert_mask;
  can_pending_alerts = 0;
  return *alerts ? ESP_OK : ESP_ERR_TIMEOUT;
}
extern "C" esp_err_t __wrap_twai_reconfigure_alerts(uint32_t enabled, uint32_t* previous) {
  Lock lock;
  if (previous) *previous = can_pending_alerts;
  can_alert_mask = enabled; return ESP_OK;
}
extern "C" esp_err_t __wrap_twai_initiate_recovery() {
  Lock lock;
  if (can_state != TWAI_STATE_BUS_OFF) return ESP_ERR_INVALID_STATE;
  can_state = esc_fault ? TWAI_STATE_RECOVERING : TWAI_STATE_STOPPED;
  if (!esc_fault) can_pending_alerts |= TWAI_ALERT_BUS_RECOVERED;
  return ESP_OK;
}
