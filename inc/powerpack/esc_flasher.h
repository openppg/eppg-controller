#ifndef POWERPACK_ESC_FLASHER_H_
#define POWERPACK_ESC_FLASHER_H_

#include <stdint.h>

enum class FlashState : uint8_t {
    IDLE,
    RESTARTING,       // Sent RestartNode, waiting for ESC to reboot
    CHECKING_BOOT,    // Sent GetBootStatus, waiting for bootloader confirmation
    STARTING,         // Sent StartFwUpgrade with header info
    SENDING_DATA,     // Sending firmware chunks via SendFwData
    ENDING,           // Sent EndFwUpgrade, waiting for confirmation
    SUCCESS,
    FAILED
};

struct FlashProgress {
    FlashState state;
    uint16_t currentChunk;
    uint16_t totalChunks;
    uint8_t retryCount;
    const char* statusMsg;
    uint8_t percentComplete;  // 0-100
};

// Initialize the flasher module (call after CAN is up)
void escFlasherInit();

// Start the firmware update process.
// fwData points to the complete firmware file (including 32-byte header).
// fwLen is the total file size in bytes.
void escFlasherStart(const uint8_t* fwData, uint32_t fwLen);

// Call this repeatedly from a task/loop to drive the state machine.
// Returns true while the process is still running.
bool escFlasherTick();

// Get current progress for display
const FlashProgress& escFlasherGetProgress();

// Check if flasher is idle (not running)
bool escFlasherIsIdle();

// Check if last flash was successful
bool escFlasherSucceeded();

#endif // POWERPACK_ESC_FLASHER_H_
