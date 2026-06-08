#ifndef UAV_CALIB_HELPERS_H
#define UAV_CALIB_HELPERS_H

#include <Arduino.h>
#include "src/common/TelemetryProtoFix.h"
#include <MAVLink_ardupilotmega.h>

// Extern declarations of UAV global states
#ifndef MAG_CAL_COMPASS_MAX
#define MAG_CAL_COMPASS_MAX 3
#endif

extern mavlink_message_t magCalProgressCache[MAG_CAL_COMPASS_MAX];
extern bool magCalProgressPending[MAG_CAL_COMPASS_MAX];
extern uint32_t magCalProgressCachedCount;
extern uint8_t magCalProgressNextId;
extern unsigned long lastMagCalProgressTxMs;
extern uint32_t magCalProgressBundleTxCount;
extern uint32_t magCalProgressBundleFailCount;
extern uint32_t magCalProgressBundleAckCount;
extern uint32_t rawTxCount;
extern uint32_t counter;
extern float lastLatency;
extern unsigned long lastAnyTx;
extern int currentSF;
extern int currentTP;

extern SX1278 radio;

// Extern functions in UAV sketch
bool calibrationConfigModeActive();
void updateMSADR(bool success);
bool waitResponseFromGCS(unsigned long timeoutMs, uint32_t expectedAckCounter, bool expectConfigAck);
unsigned long ackTimeoutForSF(uint8_t sf);

#ifndef MAG_CAL_PROGRESS_TX_INTERVAL_MS
#define MAG_CAL_PROGRESS_TX_INTERVAL_MS 70UL
#endif
#ifndef LORA_BW_KHZ
#define LORA_BW_KHZ 250.0f
#endif

#ifdef MAVLINK_MSG_ID_MAG_CAL_PROGRESS
inline void cacheMagCalProgressForGCS(const mavlink_message_t &msg) {
  mavlink_mag_cal_progress_t p;
  mavlink_msg_mag_cal_progress_decode(&msg, &p);
  uint8_t id = p.compass_id;
  if (id >= MAG_CAL_COMPASS_MAX) id = MAG_CAL_COMPASS_MAX - 1;
  magCalProgressCache[id] = msg;
  magCalProgressPending[id] = true;
  magCalProgressCachedCount++;
}

inline bool hasPendingMagCalProgress() {
  if (!calibrationConfigModeActive()) return false;
  for (uint8_t i = 0; i < MAG_CAL_COMPASS_MAX; i++) {
    if (magCalProgressPending[i]) return true;
  }
  return false;
}

inline bool buildMagCalProgressRawPacket(MavlinkRawPacket &raw, uint8_t *sentIds, uint8_t &sentCount) {
  memset(&raw, 0, sizeof(raw));
  initHeader(raw.hdr, PKT_MAVLINK_RAW);
  raw.len = 0;
  sentCount = 0;

  for (uint8_t step = 0; step < MAG_CAL_COMPASS_MAX; step++) {
    uint8_t id = (magCalProgressNextId + step) % MAG_CAL_COMPASS_MAX;
    if (!magCalProgressPending[id]) continue;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &magCalProgressCache[id]);
    if (len == 0 || len > RAW_MAVLINK_MAX) continue;
    if ((uint16_t)raw.len + len > RAW_MAVLINK_MAX) {
      if (sentCount > 0) break;
      continue;
    }

    memcpy(&raw.payload[raw.len], buf, len);
    raw.len += (uint8_t)len;
    sentIds[sentCount++] = id;
  }

  if (sentCount == 0 || raw.len == 0) return false;
  finalizePacketCrc(&raw, RAW_PKT_HEADER_LEN + raw.len);
  return true;
}

inline bool sendPendingMagCalProgressToGCS() {
  if (!hasPendingMagCalProgress()) return false;
  unsigned long now = millis();
  if (now - lastMagCalProgressTxMs < MAG_CAL_PROGRESS_TX_INTERVAL_MS) return false;
  lastMagCalProgressTxMs = now;

  MavlinkRawPacket raw;
  uint8_t sentIds[MAG_CAL_COMPASS_MAX];
  uint8_t sentCount = 0;
  if (!buildMagCalProgressRawPacket(raw, sentIds, sentCount)) return false;

  uint16_t packetLen = RAW_PKT_HEADER_LEN + raw.len;
  radio.standby();
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(currentSF);
  radio.setOutputPower(currentTP);
  int16_t txState = radio.transmit((uint8_t *)&raw, packetLen);
  rawTxCount++;
  magCalProgressBundleTxCount++;
  if (txState != RADIOLIB_ERR_NONE) {
    magCalProgressBundleFailCount++;
    updateMSADR(false);
    return true;
  }

  unsigned long t0 = millis();
  bool ackOK = waitResponseFromGCS(ackTimeoutForSF(currentSF), counter, false);
  unsigned long t1 = millis();
  if (ackOK) {
    for (uint8_t i = 0; i < sentCount; i++) {
      magCalProgressPending[sentIds[i]] = false;
    }
    magCalProgressNextId = (sentIds[sentCount - 1] + 1) % MAG_CAL_COMPASS_MAX;
    magCalProgressBundleAckCount++;
  } else {
    magCalProgressBundleFailCount++;
  }

  if (ackOK && t1 >= t0) lastLatency = (t1 - t0) / 2.0f; else lastLatency = 0.0f;
  updateMSADR(ackOK);
  lastAnyTx = millis();
  return true;
}
#endif

#endif // UAV_CALIB_HELPERS_H
