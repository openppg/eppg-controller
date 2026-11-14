#include "sp140/esc_command.h"

#include <CanardAdapter.h>
#include <SineEsc.h>

#include "driver/twai.h"

bool escQueueThrottle(bool busReady, CanardAdapter& adapter, SineEsc& esc,  // NOLINT(runtime/references)
                      EscTxStall& stall, int throttlePWM) {  // NOLINT(runtime/references)
  // libcanard has no instance until the driver is installed and esc.begin ran.
  if (!busReady) return false;
  if (!escThrottlePwmValid(throttlePWM)) return false;

  twai_status_info_t status;
  const bool driverPending = twai_get_status_info(&status) == ESP_OK &&
                             (status.msgs_to_tx > 0 || status.state != TWAI_STATE_RUNNING);
  if (stall.update(driverPending || adapter.hasPendingTx())) {
    twai_clear_transmit_queue();
    adapter.flushTx();
  }

  // Multiply by 10 to convert microseconds to the ESC's 0.1 us resolution.
  esc.setThrottleSettings2(static_cast<uint16_t>(throttlePWM * 10));
  return true;
}
