#include "../../../inc/sp140/lvgl/lvgl_task.h"
#include "../../../inc/sp140/lvgl/lvgl_display.h"

TaskHandle_t lvglTaskHandle = NULL;

void lvglTask(void* parameter) {
  // Some delay before starting to allow other tasks to initialize
  vTaskDelay(pdMS_TO_TICKS(100));

  USBSerial.println("LVGL task started");

  for (;;) {
    // Update LVGL
    updateLvgl();

    // Small delay to avoid CPU hogging
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  vTaskDelete(NULL);  // Should never reach here
}
