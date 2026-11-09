#ifndef INC_SP140_LVGL_LVGL_CLIMB_EFFICIENCY_H_
#define INC_SP140_LVGL_LVGL_CLIMB_EFFICIENCY_H_

#include <lvgl.h>
#include "../climb_efficiency.h"

// Climb-efficiency power bar (issue #57): a thin power scale under the kW
// digits with the learned level-flight power (pink tick + certainty margin),
// the best sustainable climb band (green), thermal warning/critical zones
// (yellow/red) and the current power (tick). Hidden until something has been
// learned, and whenever disarmed.

#ifndef CLIMB_EFF_BAR_DEFAULT_ENABLED
#define CLIMB_EFF_BAR_DEFAULT_ENABLED true
#endif

void setClimbEffBarEnabled(bool enabled);
bool getClimbEffBarEnabled();


// Create the (hidden) widgets. Called from setupMainScreen().
void setupClimbEfficiencyWidgets(lv_obj_t* parent, bool darkMode);

// Forget widget handles after their parent screen was deleted externally.
void clearClimbEfficiencyWidgets();

// Refresh from the estimator and the thermal-limited zones (climbBarZones).
// powerKw is the live (unfiltered) power so the current-power tick follows
// the throttle without lag.
void updateClimbEfficiencyDisplay(const ClimbEfficiencyResult& result,
                                  const ClimbBarZones& zones, float powerKw,
                                  bool armed);

#endif  // INC_SP140_LVGL_LVGL_CLIMB_EFFICIENCY_H_
