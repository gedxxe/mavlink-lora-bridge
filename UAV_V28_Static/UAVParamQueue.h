#ifndef UAV_PARAM_QUEUE_H
#define UAV_PARAM_QUEUE_H

#include <Arduino.h>
#include "TelemetryProtoFix.h"
#include <MAVLink_ardupilotmega.h>

// Extern definitions of GCS parameter sync globals
#ifndef PARAM_BULK_QUEUE_SIZE
#define PARAM_BULK_QUEUE_SIZE 12
#endif

extern ParamBulkPacket paramBulkQueue[PARAM_BULK_QUEUE_SIZE];
extern uint16_t paramBulkHead;
extern uint16_t paramBulkTail;
extern uint16_t paramBulkCount;
extern uint32_t paramBulkQueueDrop;
extern uint32_t paramValueDrop;
extern uint32_t paramBulkQueuedCount;
extern uint8_t currentSF;

// Functions for parameter bulk transfer
void flushLowQueue();

inline bool peekParamBulkPacket(ParamBulkPacket &pkt) {
  if (paramBulkCount == 0) return false;
  pkt = paramBulkQueue[paramBulkTail];
  return true;
}

inline void popParamBulkPacket() {
  if (paramBulkCount == 0) return;
  paramBulkTail = (paramBulkTail + 1) % PARAM_BULK_QUEUE_SIZE;
  paramBulkCount--;
}

inline bool startNewParamBulkPacket(uint8_t sysid, uint8_t compid) {
  if (paramBulkCount >= PARAM_BULK_QUEUE_SIZE) {
    flushLowQueue();
  }
  if (paramBulkCount >= PARAM_BULK_QUEUE_SIZE) {
    paramBulkQueueDrop++;
    paramValueDrop++;
    return false;
  }
  ParamBulkPacket &pkt = paramBulkQueue[paramBulkHead];
  memset(&pkt, 0, sizeof(pkt));
  initHeader(pkt.hdr, PKT_PARAM_BULK);
  pkt.count = 0;
  pkt.sysid = sysid;
  pkt.compid = compid;
  pkt.reserved = 0;
  finalizePacketCrc(&pkt, PARAM_BULK_LEN(0));
  paramBulkHead = (paramBulkHead + 1) % PARAM_BULK_QUEUE_SIZE;
  paramBulkCount++;
  return true;
}

inline uint8_t paramBulkRecordsLimitForCurrentSF() {
  if (currentSF == 8) return 4;
  if (currentSF == 9) return 3;
  if (currentSF >= 10) return 0;
  return PARAM_BULK_MAX_RECORDS;
}

inline bool enqueueParamValueBulkForGCS(const mavlink_message_t &msg) {
  if (msg.msgid != MAVLINK_MSG_ID_PARAM_VALUE) return false;
  mavlink_param_value_t pv;
  mavlink_msg_param_value_decode(&msg, &pv);

  uint8_t bulkLimit = paramBulkRecordsLimitForCurrentSF();
  if (bulkLimit == 0) { paramValueDrop++; return false; }

  if (paramBulkCount == 0) {
    if (!startNewParamBulkPacket(msg.sysid, msg.compid)) return false;
  }

  uint16_t lastIndex = (paramBulkHead + PARAM_BULK_QUEUE_SIZE - 1) % PARAM_BULK_QUEUE_SIZE;
  ParamBulkPacket *pkt = &paramBulkQueue[lastIndex];
  if (pkt->count >= bulkLimit || pkt->sysid != msg.sysid || pkt->compid != msg.compid) {
    if (!startNewParamBulkPacket(msg.sysid, msg.compid)) return false;
    lastIndex = (paramBulkHead + PARAM_BULK_QUEUE_SIZE - 1) % PARAM_BULK_QUEUE_SIZE;
    pkt = &paramBulkQueue[lastIndex];
  }

  CompactParamValue &r = pkt->rec[pkt->count];
  r.param_value = pv.param_value;
  r.param_count = pv.param_count;
  r.param_index = pv.param_index;
  memcpy(r.param_id, pv.param_id, 16);
  r.param_type = pv.param_type;
  pkt->count++;
  finalizePacketCrc(pkt, PARAM_BULK_LEN(pkt->count));
  paramBulkQueuedCount++;
  return true;
}

#endif // UAV_PARAM_QUEUE_H
