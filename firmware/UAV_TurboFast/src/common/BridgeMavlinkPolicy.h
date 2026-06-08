#ifndef BRIDGE_MAVLINK_POLICY_H
#define BRIDGE_MAVLINK_POLICY_H

#include <stdint.h>
#include <string.h>
#include <MAVLink_ardupilotmega.h>

// MAVLink compatibility fallbacks used by both sketches. Prefer values from
// MAVLink_ardupilotmega.h when present; these guards only protect older headers.
// Do not define MAVLINK_COMM_* here. Those names are MAVLink enum identifiers,
// not protocol constants, and macro fallbacks corrupt mavlink_types.h if this
// policy header is included before the MAVLink library headers.

#ifndef MAVLINK_MSG_ID_SET_MODE
#define MAVLINK_MSG_ID_SET_MODE 11
#endif
#ifndef MAVLINK_MSG_ID_MISSION_SET_CURRENT
#define MAVLINK_MSG_ID_MISSION_SET_CURRENT 41
#endif
#ifndef MAVLINK_MSG_ID_MISSION_CLEAR_ALL
#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL 45
#endif
#ifndef MAVLINK_MSG_ID_MANUAL_CONTROL
#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#endif
#ifndef MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70
#endif

#ifndef MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#endif

#ifndef MAV_CMD_NAV_WAYPOINT
#define MAV_CMD_NAV_WAYPOINT 16
#endif
#ifndef MAV_CMD_NAV_LOITER_UNLIM
#define MAV_CMD_NAV_LOITER_UNLIM 17
#endif
#ifndef MAV_CMD_NAV_LOITER_TURNS
#define MAV_CMD_NAV_LOITER_TURNS 18
#endif
#ifndef MAV_CMD_NAV_LOITER_TIME
#define MAV_CMD_NAV_LOITER_TIME 19
#endif
#ifndef MAV_CMD_NAV_RETURN_TO_LAUNCH
#define MAV_CMD_NAV_RETURN_TO_LAUNCH 20
#endif
#ifndef MAV_CMD_NAV_LAND
#define MAV_CMD_NAV_LAND 21
#endif
#ifndef MAV_CMD_NAV_TAKEOFF
#define MAV_CMD_NAV_TAKEOFF 22
#endif
#ifndef MAV_CMD_PREFLIGHT_CALIBRATION
#define MAV_CMD_PREFLIGHT_CALIBRATION 241
#endif
#ifndef MAV_CMD_PREFLIGHT_STORAGE
#define MAV_CMD_PREFLIGHT_STORAGE 245
#endif
#ifndef MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
#define MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN 246
#endif
#ifndef MAV_CMD_DO_SET_MODE
#define MAV_CMD_DO_SET_MODE 176
#endif
#ifndef MAV_CMD_DO_SET_HOME
#define MAV_CMD_DO_SET_HOME 179
#endif
#ifndef MAV_CMD_DO_REPOSITION
#define MAV_CMD_DO_REPOSITION 192
#endif
#ifndef MAV_CMD_DO_MOTOR_TEST
#define MAV_CMD_DO_MOTOR_TEST 209
#endif
#ifndef MAV_CMD_MISSION_START
#define MAV_CMD_MISSION_START 300
#endif
#ifndef MAV_CMD_COMPONENT_ARM_DISARM
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#endif
#ifndef MAV_CMD_SET_MESSAGE_INTERVAL
#define MAV_CMD_SET_MESSAGE_INTERVAL 511
#endif
#ifndef MAV_CMD_REQUEST_MESSAGE
#define MAV_CMD_REQUEST_MESSAGE 512
#endif
#ifndef MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
#define MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES 520
#endif

#ifndef MAV_SEVERITY_CRITICAL
#define MAV_SEVERITY_CRITICAL 2
#endif
#ifndef MAV_SEVERITY_WARNING
#define MAV_SEVERITY_WARNING 4
#endif
#ifndef MAV_SEVERITY_NOTICE
#define MAV_SEVERITY_NOTICE 5
#endif
#ifndef MAV_SEVERITY_INFO
#define MAV_SEVERITY_INFO 6
#endif

#ifndef MAV_RESULT_ACCEPTED
#define MAV_RESULT_ACCEPTED 0
#endif
#ifndef MAV_RESULT_DENIED
#define MAV_RESULT_DENIED 2
#endif
#ifndef MAV_RESULT_FAILED
#define MAV_RESULT_FAILED 4
#endif

// ArduPilot command aliases used by existing bridge logic and Mission Planner
// setup flows. Keep these centralized so GCS and UAV classify commands equally.
#define CMD_DO_SET_MODE               MAV_CMD_DO_SET_MODE
#define CMD_COMPONENT_ARM_DISARM      MAV_CMD_COMPONENT_ARM_DISARM
#define CMD_PREFLIGHT_CALIBRATION     MAV_CMD_PREFLIGHT_CALIBRATION
#define CMD_PREFLIGHT_STORAGE         MAV_CMD_PREFLIGHT_STORAGE
#define CMD_PREFLIGHT_REBOOT_SHUTDOWN MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
#define CMD_START_RX_PAIR             500
#define CMD_DO_START_MAG_CAL          42424
#define CMD_DO_ACCEPT_MAG_CAL         42425
#define CMD_DO_CANCEL_MAG_CAL         42426
#define CMD_ACCELCAL_VEHICLE_POS      42429

static inline bool bridgeParamIdStartsWith(const char *id, const char *prefix) {
  return id != nullptr && prefix != nullptr && strncmp(id, prefix, strlen(prefix)) == 0;
}

static inline bool bridgeParamIdEquals(const char *id, const char *name) {
  return id != nullptr && name != nullptr && strncmp(id, name, 16) == 0;
}

static inline bool bridgeIsInteractiveSetupParamId(const char *id) {
  if (id == nullptr || id[0] == 0) return false;
  if (bridgeParamIdStartsWith(id, "COMPASS_")) return true;
  if (bridgeParamIdStartsWith(id, "INS_")) return true;
  if (bridgeParamIdStartsWith(id, "AHRS_")) return true;
  if (bridgeParamIdStartsWith(id, "GPS_")) return true;
  if (bridgeParamIdStartsWith(id, "GPS2_")) return true;
  if (bridgeParamIdStartsWith(id, "SERIAL")) return true;
  if (bridgeParamIdStartsWith(id, "CAN_")) return true;
  if (bridgeParamIdStartsWith(id, "BRD_")) return true;
  if (bridgeParamIdStartsWith(id, "EK2_")) return true;
  if (bridgeParamIdStartsWith(id, "EK3_")) return true;
  if (bridgeParamIdStartsWith(id, "ARMING_")) return true;
  if (bridgeParamIdStartsWith(id, "RC")) return true;
  if (bridgeParamIdStartsWith(id, "FS_")) return true;
  if (bridgeParamIdStartsWith(id, "SERVO")) return true;
  if (bridgeParamIdStartsWith(id, "MOT_")) return true;
  if (bridgeParamIdStartsWith(id, "ESC_")) return true;
  if (bridgeParamIdStartsWith(id, "FLTMODE")) return true;
  if (bridgeParamIdEquals(id, "SIMPLE")) return true;
  if (bridgeParamIdEquals(id, "SUPER_SIMPLE")) return true;
  if (bridgeParamIdEquals(id, "MODE_CH")) return true;
  if (bridgeParamIdStartsWith(id, "BATT")) return true;
  if (bridgeParamIdStartsWith(id, "FENCE_")) return true;
  if (bridgeParamIdStartsWith(id, "ATC_")) return true;
  if (bridgeParamIdStartsWith(id, "PSC_")) return true;
  if (bridgeParamIdStartsWith(id, "PILOT_")) return true;
  if (bridgeParamIdStartsWith(id, "WPNAV_")) return true;
  if (bridgeParamIdStartsWith(id, "ANGLE_")) return true;
  return false;
}

static inline bool bridgeIsCalibrationCommand(uint16_t command) {
  return command == CMD_PREFLIGHT_CALIBRATION ||
         command == CMD_DO_START_MAG_CAL ||
         command == CMD_DO_ACCEPT_MAG_CAL ||
         command == CMD_DO_CANCEL_MAG_CAL ||
         command == CMD_ACCELCAL_VEHICLE_POS;
}

static inline bool bridgeIsSetupConfigCommand(uint16_t command) {
  return bridgeIsCalibrationCommand(command) ||
         command == CMD_PREFLIGHT_STORAGE ||
         command == CMD_PREFLIGHT_REBOOT_SHUTDOWN ||
         command == CMD_START_RX_PAIR;
}

static inline bool bridgeIsFlightActionCommand(uint16_t command) {
  return command == CMD_COMPONENT_ARM_DISARM ||
         command == CMD_DO_SET_MODE ||
         command == MAV_CMD_NAV_WAYPOINT ||
         command == MAV_CMD_NAV_LOITER_UNLIM ||
         command == MAV_CMD_NAV_LOITER_TURNS ||
         command == MAV_CMD_NAV_LOITER_TIME ||
         command == MAV_CMD_NAV_RETURN_TO_LAUNCH ||
         command == MAV_CMD_NAV_LAND ||
         command == MAV_CMD_NAV_TAKEOFF ||
         command == MAV_CMD_DO_REPOSITION ||
         command == MAV_CMD_MISSION_START ||
         command == MAV_CMD_DO_SET_HOME ||
         command == MAV_CMD_DO_MOTOR_TEST;
}

#endif // BRIDGE_MAVLINK_POLICY_H
