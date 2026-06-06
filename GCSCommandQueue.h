#ifndef GCS_COMMAND_QUEUE_H
#define GCS_COMMAND_QUEUE_H

#include <Arduino.h>
#include "TelemetryProtoFix.h"
#include <MAVLink_ardupilotmega.h>

// Extern references to variables in GCS sketch
#ifndef COMPACT_CMD_QUEUE_SIZE
#define COMPACT_CMD_QUEUE_SIZE 12
#endif

extern CompactCommandQueueItem compactCmdQueue[COMPACT_CMD_QUEUE_SIZE];
extern uint16_t compactCmdHead;
extern uint16_t compactCmdTail;
extern uint16_t compactCmdCount;
extern uint32_t compactCmdDrop;
extern uint32_t commandDrop;
extern uint32_t compactCmdQueuedCount;
extern uint16_t compactCmdSeq;
extern int activeSF;

// Forward declared functions in GCS sketch
bool isFlightActionGCSMessage(const mavlink_message_t &msg);
bool isFlightActionCommandId(uint16_t commandId);

inline int32_t compactScale1000(float v) { return (int32_t)(v * COMPACT_CMD_SCALE_1000 + (v >= 0 ? 0.5f : -0.5f)); }
inline int32_t compactScale1e7(float v) { return (int32_t)(v * COMPACT_CMD_SCALE_1E7 + (v >= 0 ? 0.5f : -0.5f)); }
inline float compactFromX1000(int32_t v) { return ((float)v) / COMPACT_CMD_SCALE_1000; }

inline bool enqueueCompactPacket(const void *packet, uint8_t len, uint16_t command) {
  if (len == 0 || len > COMPACT_CMD_MAX_LEN) { compactCmdDrop++; commandDrop++; return false; }
  if (compactCmdCount >= COMPACT_CMD_QUEUE_SIZE) {
    compactCmdTail = (compactCmdTail + 1) % COMPACT_CMD_QUEUE_SIZE;
    compactCmdCount--; compactCmdDrop++;
  }
  CompactCommandQueueItem &item = compactCmdQueue[compactCmdHead];
  memset(&item, 0, sizeof(item));
  item.len = len;
  item.command = command;
  memcpy(item.payload, packet, len);
  compactCmdHead = (compactCmdHead + 1) % COMPACT_CMD_QUEUE_SIZE;
  compactCmdCount++; compactCmdQueuedCount++;
  return true;
}

inline bool peekCompactCommandPacket(CompactCommandQueueItem &item) {
  if (compactCmdCount == 0) return false;
  item = compactCmdQueue[compactCmdTail];
  return true;
}

inline void popCompactCommandPacket() {
  if (compactCmdCount == 0) return;
  compactCmdTail = (compactCmdTail + 1) % COMPACT_CMD_QUEUE_SIZE;
  compactCmdCount--;
}

inline void flushCompactCommandQueue() { compactCmdHead = 0; compactCmdTail = 0; compactCmdCount = 0; }

inline bool shouldUseCompactCommandForMessage(const mavlink_message_t &msg) {
  if (activeSF < 8) return false;
  if (!isFlightActionGCSMessage(msg)) return false;
  return msg.msgid == MAVLINK_MSG_ID_SET_MODE || msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG;
}

inline bool enqueueCompactCommandFromMissionPlanner(const mavlink_message_t &msg, uint16_t commandIdForAck) {
  if (!shouldUseCompactCommandForMessage(msg)) return false;
  if (msg.msgid == MAVLINK_MSG_ID_SET_MODE) {
    mavlink_set_mode_t sm; mavlink_msg_set_mode_decode(&msg, &sm);
    CompactSetModePacket pkt = {};
    initHeader(pkt.hdr, PKT_CMD_COMPACT);
    pkt.seq = ++compactCmdSeq;
    pkt.kind = COMPACT_CMD_KIND_SET_MODE;
    pkt.target_system = sm.target_system;
    pkt.base_mode = sm.base_mode;
    pkt.custom_mode = sm.custom_mode;
    finalizePacketCrc(&pkt, sizeof(pkt));
    return enqueueCompactPacket(&pkt, sizeof(pkt), commandIdForAck ? commandIdForAck : CMD_DO_SET_MODE);
  }

  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd; mavlink_msg_command_long_decode(&msg, &cmd);
    uint16_t command = (uint16_t)cmd.command;
    if (command == CMD_COMPONENT_ARM_DISARM) {
      CompactArmDisarmPacket pkt = {};
      initHeader(pkt.hdr, PKT_CMD_COMPACT);
      pkt.seq = ++compactCmdSeq;
      pkt.kind = COMPACT_CMD_KIND_ARM_DISARM;
      pkt.target_system = cmd.target_system;
      pkt.target_component = cmd.target_component;
      pkt.arm = (cmd.param1 >= 0.5f) ? 1 : 0;
      pkt.param2_x1000 = compactScale1000(cmd.param2);
      finalizePacketCrc(&pkt, sizeof(pkt));
      return enqueueCompactPacket(&pkt, sizeof(pkt), command);
    }
    if (isFlightActionCommandId(command)) {
      CompactCommandLongPacket pkt = {};
      initHeader(pkt.hdr, PKT_CMD_COMPACT);
      pkt.seq = ++compactCmdSeq;
      pkt.kind = COMPACT_CMD_KIND_COMMAND_LONG;
      pkt.command = command;
      pkt.target_system = cmd.target_system;
      pkt.target_component = cmd.target_component;
      pkt.confirmation = cmd.confirmation;
      pkt.p1_x1000 = compactScale1000(cmd.param1);
      pkt.p2_x1000 = compactScale1000(cmd.param2);
      pkt.p3_x1000 = compactScale1000(cmd.param3);
      pkt.p4_x1000 = compactScale1000(cmd.param4);
      pkt.p5_x1e7 = compactScale1e7(cmd.param5);
      pkt.p6_x1e7 = compactScale1e7(cmd.param6);
      pkt.p7_x1000 = compactScale1000(cmd.param7);
      finalizePacketCrc(&pkt, sizeof(pkt));
      return enqueueCompactPacket(&pkt, sizeof(pkt), command);
    }
  }
  return false;
}

#endif // GCS_COMMAND_QUEUE_H
