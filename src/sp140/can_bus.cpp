#include "sp140/can_bus.h"
#include "sp140/bms.h"
#include "sp140/esc.h"
#include "sp140/globals.h"
#include "driver/twai.h"

TaskHandle_t canBusTaskHandle = NULL;

/**
 * Initialize the unified CAN bus management system
 * This should be called after both ESC and BMS systems are initialized
 */
void initUnifiedCANBus() {
  USBSerial.println("Initializing unified CAN bus management...");

  // Create the unified CAN bus task with high priority
  xTaskCreatePinnedToCore(
    unifiedCANBusTask,
    "UnifiedCANBus",
    4096,
    NULL,
    6,  // High priority to ensure messages are processed quickly
    &canBusTaskHandle,
    1   // Pin to core 1
  );

  USBSerial.println("Unified CAN bus task created successfully");
}

/**
 * Unified CAN bus task that handles all TWAI message processing
 * Routes messages to appropriate systems (BMS or ESC) based on message ID
 */
void unifiedCANBusTask(void* parameter) {
  twai_message_t message;
  const TickType_t maxWaitTime = pdMS_TO_TICKS(1); // 1ms timeout

  USBSerial.println("Unified CAN bus task started");

  while (true) {
    // Only process if TWAI is initialized
    if (!escTwaiInitialized) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

        // Process all available messages
    while (twai_receive(&message, 0) == ESP_OK) {
      processCANMessage(message);
    }

    // Process ESC tasks (alerts, transmission, stale transfers)
    processESCTasks();

    // Small delay to prevent task from consuming too much CPU
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

/**
 * Process a single CAN message and route it to the appropriate system
 * @param message The received TWAI message
 */
void processCANMessage(const twai_message_t& message) {
  // Check if this is a BMS message
  if (isBMSMessage(message.identifier)) {
    // Route to BMS system
    parseBMSPacket(&message);

    #ifdef CAN_DEBUG
    USBSerial.printf("BMS message processed: ID=0x%08X\n", message.identifier);
    #endif
    } else {
    // Route to ESC system
    processESCMessage(message);

    #ifdef CAN_DEBUG
    USBSerial.printf("ESC message processed: ID=0x%08X\n", message.identifier);
    #endif
  }
}
