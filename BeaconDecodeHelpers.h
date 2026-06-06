#ifndef BEACON_DECODE_HELPERS_H
#define BEACON_DECODE_HELPERS_H

#include <stdint.h>
#include <Arduino.h>
#include "TelemetryProtoFix.h"
#include <MAVLink_ardupilotmega.h>

// Forward declarations of GCS global states
extern bool mpTelemetryOutputContext;
extern bool mpLinkLossState;
extern bool mpLinkLossAnnounced;
extern unsigned long lastMpLinkLossZeroStatusMs;
extern uint8_t optimisticModeBaseMode;
extern uint32_t optimisticModeCustomMode;
extern uint8_t optimisticModeSystemStatus;
extern mavlink_message_t msg_out;
extern bool latestBeaconValid;
extern PixhawkDataBeacon latestBeaconData;
extern unsigned long lastRadioStatusMs;
extern unsigned long lastSyntheticHeartbeatMs;

// Forward declarations of GCS configuration macros and constants
#ifndef HEARTBEAT_SYNTH_INTERVAL_MS
#define HEARTBEAT_SYNTH_INTERVAL_MS 1000UL
#endif
#ifndef RADIO_STATUS_INTERVAL_MS
#define RADIO_STATUS_INTERVAL_MS 1000UL
#endif

// Forward declarations of GCS functions
bool isUplinkFreshForMissionPlanner();
void enterMissionPlannerLinkLostState(unsigned long now);
void serviceMissionPlannerLostRadioStatus(unsigned long now);
void announceMissionPlannerLinkRecoveredIfNeeded();
bool optimisticModeOverrideActive();
void sendHeartbeatFromData(uint8_t sysid, uint8_t compid, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
void sendAttitudeFromData(uint8_t sysid, uint8_t compid, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
void sendGlobalPositionFromData(uint8_t sysid, uint8_t compid, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg);
void sendGpsRawFromBeacon(uint8_t sysid, uint8_t compid, const PixhawkDataBeacon &d);
void sendSysStatusFromData(uint8_t sysid, uint8_t compid, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining);
void sendVfrHudFromData(uint8_t sysid, uint8_t compid, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float climb);
void sendEkfStatusFromData(uint8_t sysid, uint8_t compid, uint16_t ekf_flags);
void sendMavlinkMessageToMissionPlanner(const mavlink_message_t &msg);
void sendRadioStatusToMissionPlanner(uint8_t sysid, uint8_t compid);

// Math conversion helpers on GCS side
static inline float centiDegToRad(int16_t cd) {
  return ((float)cd) * PI / 18000.0f;
}

static inline int32_t decimeterToMillimeter(int16_t dm) {
  return (int32_t)dm * 100L;
}

static inline void sendTelemetryBeaconToMissionPlanner(const PixhawkDataBeacon &d) {
  bool oldTelemetryContext = mpTelemetryOutputContext;
  mpTelemetryOutputContext = true;

  if (!isUplinkFreshForMissionPlanner()) {
    unsigned long now = millis();
    enterMissionPlannerLinkLostState(now);
    serviceMissionPlannerLostRadioStatus(now);
    mpTelemetryOutputContext = oldTelemetryContext;
    return;
  }
  if (mpLinkLossState || mpLinkLossAnnounced) {
    announceMissionPlannerLinkRecoveredIfNeeded();
  }
  mpLinkLossState = false;
  lastMpLinkLossZeroStatusMs = 0;
  uint8_t sysid = d.system_id ? d.system_id : 1;
  uint8_t compid = d.component_id ? d.component_id : MAV_COMP_ID_AUTOPILOT1;
  uint8_t hbBaseMode = d.base_mode;
  uint32_t hbCustomMode = d.custom_mode;
  uint8_t hbSystemStatus = d.system_status;
  if (optimisticModeOverrideActive()) {
    hbBaseMode = optimisticModeBaseMode;
    hbCustomMode = optimisticModeCustomMode;
    hbSystemStatus = optimisticModeSystemStatus;
  }
  sendHeartbeatFromData(sysid, compid, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, hbBaseMode, hbCustomMode, hbSystemStatus);
  if (d.valid_flags & VALID_ATTITUDE) sendAttitudeFromData(sysid, compid, centiDegToRad(d.roll_cd), centiDegToRad(d.pitch_cd), centiDegToRad(d.yaw_cd), 0, 0, 0);
  if (d.valid_flags & VALID_GLOBAL_POS) {
    // Pass real NED velocity from beacon (vx_cms/vy_cms/vz_cms are cm/s NED)
    sendGlobalPositionFromData(sysid, compid,
                               d.lat, d.lon,
                               decimeterToMillimeter(d.gps.alt_dm),
                               decimeterToMillimeter(d.relative_alt_dm),
                               d.vx_cms, d.vy_cms, d.vz_cms, d.heading);
  }
  if (d.valid_flags & VALID_GPS_RAW) sendGpsRawFromBeacon(sysid, compid, d);
  if (d.valid_flags & VALID_SYS_STATUS) sendSysStatusFromData(sysid, compid, d.voltage_battery, d.current_battery, d.battery_remaining);
  float airspeed_ms    = d.airspeed_cms   / 100.0f;
  float groundspeed_ms = d.groundspeed_cms / 100.0f;
  int16_t heading_deg  = (int16_t)(d.heading / 100);
  float climb_ms       = d.climb_cms / 100.0f;
  sendVfrHudFromData(sysid, compid, airspeed_ms, groundspeed_ms, heading_deg, d.throttle, climb_ms);
  #ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
  if (d.valid_flags & VALID_EKF) sendEkfStatusFromData(sysid, compid, d.ekf_flags);
  #endif

  #ifdef MAVLINK_MSG_ID_VIBRATION
  // Reconstruct VIBRATION message from compacted beacon fields
  if (d.valid_flags2 & VALID2_VIBRATION) {
    mavlink_vibration_t vib = {};
    vib.time_usec   = (uint64_t)millis() * 1000ULL;
    vib.vibration_x = (float)d.vibe_x_x100 / 100.0f;
    vib.vibration_y = (float)d.vibe_y_x100 / 100.0f;
    vib.vibration_z = (float)d.vibe_z_x100 / 100.0f;
    vib.clipping_0  = d.accel_clip;   // total clip count packed into clipping_0
    vib.clipping_1  = 0;
    vib.clipping_2  = 0;
    mavlink_msg_vibration_encode(sysid, compid, &msg_out, &vib);
    sendMavlinkMessageToMissionPlanner(msg_out);
  }
  #endif

  #ifdef MAVLINK_MSG_ID_RC_CHANNELS
  // Reconstruct RC_CHANNELS message from compacted channel percentage bytes
  if (d.valid_flags2 & VALID2_RC_CHANNELS) {
    mavlink_rc_channels_t rc = {};
    rc.time_boot_ms = millis();
    rc.chancount    = 16;
    rc.chan1_raw  = rcPctToRaw(d.rc_ch_pct[0]);  rc.chan2_raw  = rcPctToRaw(d.rc_ch_pct[1]);
    rc.chan3_raw  = rcPctToRaw(d.rc_ch_pct[2]);  rc.chan4_raw  = rcPctToRaw(d.rc_ch_pct[3]);
    rc.chan5_raw  = rcPctToRaw(d.rc_ch_pct[4]);  rc.chan6_raw  = rcPctToRaw(d.rc_ch_pct[5]);
    rc.chan7_raw  = rcPctToRaw(d.rc_ch_pct[6]);  rc.chan8_raw  = rcPctToRaw(d.rc_ch_pct[7]);
    rc.chan9_raw  = rcPctToRaw(d.rc_ch_pct[8]);  rc.chan10_raw = rcPctToRaw(d.rc_ch_pct[9]);
    rc.chan11_raw = rcPctToRaw(d.rc_ch_pct[10]); rc.chan12_raw = rcPctToRaw(d.rc_ch_pct[11]);
    rc.chan13_raw = rcPctToRaw(d.rc_ch_pct[12]); rc.chan14_raw = rcPctToRaw(d.rc_ch_pct[13]);
    rc.chan15_raw = rcPctToRaw(d.rc_ch_pct[14]); rc.chan16_raw = rcPctToRaw(d.rc_ch_pct[15]);
    rc.rssi = 255;
    mavlink_msg_rc_channels_encode(sysid, compid, &msg_out, &rc);
    sendMavlinkMessageToMissionPlanner(msg_out);
  }
  #endif

  sendRadioStatusToMissionPlanner(sysid, compid);
  lastRadioStatusMs = millis();
  mpTelemetryOutputContext = oldTelemetryContext;
}

static inline void sendSyntheticHeartbeatToMissionPlannerIfNeeded() {
  bool oldTelemetryContext = mpTelemetryOutputContext;
  mpTelemetryOutputContext = true;
  if (!latestBeaconValid || !isUplinkFreshForMissionPlanner()) { mpTelemetryOutputContext = oldTelemetryContext; return; }
  unsigned long now = millis();
  if (now - lastSyntheticHeartbeatMs < HEARTBEAT_SYNTH_INTERVAL_MS) { mpTelemetryOutputContext = oldTelemetryContext; return; }
  uint8_t sysid = latestBeaconData.system_id ? latestBeaconData.system_id : 1;
  uint8_t compid = latestBeaconData.component_id ? latestBeaconData.component_id : MAV_COMP_ID_AUTOPILOT1;
  uint8_t hbBaseMode = latestBeaconData.base_mode;
  uint32_t hbCustomMode = latestBeaconData.custom_mode;
  uint8_t hbSystemStatus = latestBeaconData.system_status;
  if (optimisticModeOverrideActive()) {
    hbBaseMode = optimisticModeBaseMode;
    hbCustomMode = optimisticModeCustomMode;
    hbSystemStatus = optimisticModeSystemStatus;
  }
  sendHeartbeatFromData(sysid, compid, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                        hbBaseMode, hbCustomMode, hbSystemStatus);
  if (now - lastRadioStatusMs >= RADIO_STATUS_INTERVAL_MS) {
    sendRadioStatusToMissionPlanner(sysid, compid);
    lastRadioStatusMs = now;
  }
  lastSyntheticHeartbeatMs = now;
  mpTelemetryOutputContext = oldTelemetryContext;
}

#endif // BEACON_DECODE_HELPERS_H
