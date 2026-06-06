#ifndef GC_CALIB_HELPERS_H
#define GC_CALIB_HELPERS_H

#include <Arduino.h>
#include "TelemetryProtoFix.h"

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
#ifndef LINK_MODE_CALIBRATION
#define LINK_MODE_CALIBRATION 2
#endif
#ifndef LINK_MODE_PARAM_SYNC
#define LINK_MODE_PARAM_SYNC 1
#endif
#ifndef LINK_MODE_NORMAL
#define LINK_MODE_NORMAL 0
#endif

static inline bool calibrationConfigModeActive() {
  return calConfigActive && millis() < calConfigUntilMs;
}

static inline void startCalibrationConfigMode() {
  calConfigActive = true;
  calConfigUntilMs = millis() + CAL_CONFIG_HOLD_MS;
  paramModeHoldUntilMs = millis() + PARAM_MODE_HOLD_MS;
  linkMode = LINK_MODE_CALIBRATION;
}

static inline void stopCalibrationConfigMode() {
  calConfigActive = false;
  calConfigUntilMs = 0;
  if (!paramSyncActive && millis() > paramModeHoldUntilMs) linkMode = LINK_MODE_NORMAL;
}

static inline void updateCalibrationConfigMode() {
  if (!calConfigActive) return;
  if (millis() <= calConfigUntilMs) return;
  stopCalibrationConfigMode();
}

static inline bool fastOperationalModeActive() {
  return paramSyncActive || calibrationConfigModeActive() || millis() < paramModeHoldUntilMs || linkMode == LINK_MODE_PARAM_SYNC || linkMode == LINK_MODE_CALIBRATION;
}

#endif // GC_CALIB_HELPERS_H
