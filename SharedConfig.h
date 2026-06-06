#ifndef SHARED_CONFIG_H
#define SHARED_CONFIG_H

#include <stdint.h>

#ifndef UINT8_MAX
#define UINT8_MAX 255
#endif

#ifndef RADIOLIB_ERR_UNKNOWN
#define RADIOLIB_ERR_UNKNOWN -999
#endif

#ifndef MAVLINK_COMM_2
#define MAVLINK_COMM_2 2
#endif

// ================= Protocol limits used in structs =================
#define RAW_MAVLINK_MAX       235
#define PARAM_BULK_MAX_RECORDS 9
#define COMPACT_CMD_MAX_LEN   44  // We'll leave the sizeof calculation below or use a macro

// Include after protocol limits so it can use them

// ================= MAVLink Message IDs =================
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

// Mission Planner / ArduPilot command IDs.
#define CMD_DO_SET_MODE               176
#ifndef MAV_CMD_DO_SET_MODE
#define MAV_CMD_DO_SET_MODE 176
#endif
#define CMD_COMPONENT_ARM_DISARM      400
#ifndef MAV_CMD_COMPONENT_ARM_DISARM
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#endif
#define CMD_PREFLIGHT_CALIBRATION     241
#define CMD_PREFLIGHT_STORAGE         245
#define CMD_PREFLIGHT_REBOOT_SHUTDOWN 246
#define CMD_START_RX_PAIR             500
#define CMD_DO_START_MAG_CAL          42424
#define CMD_DO_ACCEPT_MAG_CAL         42425
#define CMD_DO_CANCEL_MAG_CAL         42426
#define CMD_ACCELCAL_VEHICLE_POS      42429
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
#ifndef MAV_CMD_DO_REPOSITION
#define MAV_CMD_DO_REPOSITION 192
#endif
#ifndef MAV_CMD_MISSION_START
#define MAV_CMD_MISSION_START 300
#endif
#ifndef MAV_CMD_DO_SET_HOME
#define MAV_CMD_DO_SET_HOME 179
#endif

#ifndef MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#endif

// ================= Link Mode =================
#define LINK_MODE_NORMAL       0
#define LINK_MODE_PARAM_SYNC   1
#define LINK_MODE_CALIBRATION  2
#define LINK_MODE_MISSION      3
#define LINK_MODE_FAILSAFE     4
#define LINK_MODE_COMMAND      5

// ================= Protocol =================
#define PROTOCOL_VERSION       0x04
#define NETWORK_ID             0x2244
#define AUTH_TOKEN             0xA56C93D1UL
#define SECURITY_KEY_MIX        0x3D7F21B9UL
#define SECURITY_ANTI_REPLAY_ENABLE 1
#define SECURITY_MAX_COUNTER_GAP    5000UL
#define SECURITY_REBOOT_GRACE_MS    15000UL

// ================= LoRa Common =================
#define LORA_SS    5
#define LORA_RST   25
#define LORA_DIO0  26
#define LORA_DIO1  RADIOLIB_NC

#define FREQ_MHZ      433.0
#define LORA_BW_KHZ   500.0          // BW 500 kHz
#define LORA_CR_DEN   8
#define LORA_SYNC     0x12

#define SF_MIN 7
#define SF_MAX 12
#define TP_MIN 10
#define TP_MAX 16

#define LORA_PREAMBLE_SYMBOLS 8.0f // Usually overridden or handled per side, but we'll set a default
#define LORA_PHY_CRC_ENABLED 1
#define LORA_IMPLICIT_HEADER 0

#define LORA_INIT_RETRY_COUNT 8
#define LORA_INIT_RETRY_DELAY_MS 250

// ================= Packet Types =================
#define PKT_LINK_ACK          0xA5
#define PKT_MAVLINK_RAW       0x4D
#define PKT_CMD_COMPACT       0x43
#define PKT_PARAM_BULK        0x50
#define PKT_TELEM_BEACON      0x57
#define PKT_CONFIG_PROPOSE    0xC0
#define PKT_CONFIG_ACK        0xC1

#define PROFILE_BEACON        0

// ================= Valid Flags =================
#define VALID_HEARTBEAT   (1UL << 0)
#define VALID_ATTITUDE    (1UL << 1)
#define VALID_GLOBAL_POS  (1UL << 2)
#define VALID_VFR_HUD     (1UL << 3)
#define VALID_SYS_STATUS  (1UL << 4)
#define VALID_EKF         (1UL << 5)
#define VALID_GPS_RAW     (1UL << 6)

#endif
