#ifndef POWERPACK_ESC_CONFIG_H_
#define POWERPACK_ESC_CONFIG_H_

#include <stdint.h>

enum class ConfigState : uint8_t {
    IDLE,
    WRITING_PARAMS,
    SAVING,           // ExecuteOpcode SAVE
    RESTARTING,       // RestartNode after save
    SUCCESS,
    FAILED
};

struct ConfigProgress {
    ConfigState state;
    uint16_t currentParam;
    uint16_t totalParams;
    const char* currentParamName;
    const char* statusMsg;
    uint8_t percentComplete;
};

// Initialize the config writer (call after CAN is up)
void escConfigInit();

// Start writing the production config to the ESC.
// Uses the embedded config table.
void escConfigStart();

// Drive the config writer state machine. Returns true while running.
bool escConfigTick();

// Get current progress
const ConfigProgress& escConfigGetProgress();

bool escConfigSucceeded();

#endif // POWERPACK_ESC_CONFIG_H_
