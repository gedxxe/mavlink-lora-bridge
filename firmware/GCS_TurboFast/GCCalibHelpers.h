#ifndef GC_CALIB_HELPERS_H
#define GC_CALIB_HELPERS_H

#include <Arduino.h>
#include "src/common/TelemetryProtoFix.h"
#include "src/common/LinkProfile.h"

// Extern references to variables in GCS sketch
extern bool calConfigActive;
extern unsigned long calConfigUntilMs;
extern unsigned long paramModeHoldUntilMs;
extern uint8_t linkMode;
extern bool paramSyncActive;

#ifndef CAL_CONFIG_HOLD_MS
#define CAL_CONFIG_HOLD_MS 180000UL
#endif
#ifndef PARAM_MODE_HOLD_MS
#define PARAM_MODE_HOLD_MS 8000UL
#endif
inline bool calibrationConfigModeActive() {
  return calConfigActive && millis() < calConfigUntilMs;
}

inline void startCalibrationConfigMode() {
  calConfigActive = true;
  calConfigUntilMs = millis() + CAL_CONFIG_HOLD_MS;
  paramModeHoldUntilMs = millis() + PARAM_MODE_HOLD_MS;
  linkMode = LINK_MODE_CALIBRATION;
}

inline void stopCalibrationConfigMode() {
  calConfigActive = false;
  calConfigUntilMs = 0;
  if (!paramSyncActive && millis() > paramModeHoldUntilMs) linkMode = LINK_MODE_NORMAL;
}

inline void updateCalibrationConfigMode() {
  if (!calConfigActive) return;
  if (millis() <= calConfigUntilMs) return;
  stopCalibrationConfigMode();
}

inline bool fastOperationalModeActive() {
  return paramSyncActive || calibrationConfigModeActive() || millis() < paramModeHoldUntilMs || linkMode == LINK_MODE_PARAM_SYNC || linkMode == LINK_MODE_CALIBRATION;
}

#endif // GC_CALIB_HELPERS_H
