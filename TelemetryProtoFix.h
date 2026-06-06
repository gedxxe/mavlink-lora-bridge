#ifndef TELEMETRY_PROTO_FIX_H
#define TELEMETRY_PROTO_FIX_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

// Forward declarations for Arduino IDE automatic prototype generation.
struct PacketHeader;
struct GpsRawData;
struct GpsDataRingkas;
struct TelemetryMeta;
struct PixhawkDataBeacon;
struct TelemetryBeaconPacket;
struct MavlinkRawPacket;
struct CompactCommandBasePacket;
struct CompactArmDisarmPacket;
struct CompactSetModePacket;
struct CompactCommandLongPacket;
struct CompactCommandQueueItem;
struct CompactParamValue;
struct ParamBulkPacket;
struct LinkAckPacket;
struct ConfigProposalPacket;
struct ConfigAckPacket;

// ================= Protocol Constants =================
#define PROTOCOL_VERSION            0x04
#define NETWORK_ID                  0x2244
#define AUTH_TOKEN                  0xA56C93D1UL  // Pre-shared secret used for dynamically generated auth-tag per packet.
#define SECURITY_KEY_MIX            0x3D7F21B9UL
#define SECURITY_ANTI_REPLAY_ENABLE 1
#define SECURITY_MAX_COUNTER_GAP    5000UL
#define SECURITY_REBOOT_GRACE_MS    15000UL

// ================= Packet Types =================
#define PKT_LINK_ACK          0xA5
#define PKT_MAVLINK_RAW       0x4D
#define PKT_CMD_COMPACT       0x43
#define PKT_PARAM_BULK        0x50
#define PKT_TELEM_BEACON      0x57
#define PKT_CONFIG_PROPOSE    0xC0
#define PKT_CONFIG_ACK        0xC1

// ================= Other Constants =================
#define PROFILE_BEACON               0
#define RAW_MAVLINK_MAX              235
#define PARAM_BULK_MAX_RECORDS       9
#define LORA_RX_MAX                  255
#define COMPACT_CMD_KIND_ARM_DISARM  1
#define COMPACT_CMD_KIND_SET_MODE    2
#define COMPACT_CMD_KIND_COMMAND_LONG 3
#define COMPACT_CMD_SCALE_1000       1000.0f
#define COMPACT_CMD_SCALE_1E7        10000000.0f
#define LORA_PHY_CRC_ENABLED         1
#define LORA_IMPLICIT_HEADER         0

// ================= Structure Definitions =================

struct __attribute__((packed)) PacketHeader {
  uint8_t type;
  uint8_t protocol;
  uint16_t network_id;
  uint16_t crc;
  uint32_t token;
};

struct __attribute__((packed)) GpsRawData {
  uint64_t time_usec;
  uint8_t fix_type;
  int32_t lat;
  int32_t lon;
  int32_t alt_mm;
  uint16_t eph;
  uint16_t epv;
  uint16_t vel;
  uint16_t cog;
  uint8_t satellites_visible;
};
typedef GpsRawData GpsRawDataFull; // Alias for compatibility with UAV sketch

struct __attribute__((packed)) GpsDataRingkas {
  uint8_t fix_type;
  uint8_t satellites_visible;
  int16_t alt_dm;      // Alt in decimeter; GCS scales it back to mm.
  uint16_t eph;
  uint16_t epv;
};

struct __attribute__((packed)) TelemetryMeta {
  uint8_t cnt_ack;
  uint16_t success_streak;
  uint16_t fail_streak;
  uint32_t telem_tx_energy_mJ_x100;
};

struct __attribute__((packed)) PixhawkDataBeacon {
  uint8_t valid_flags;
  uint8_t system_id;
  uint8_t component_id;
  uint8_t base_mode;
  uint32_t custom_mode;
  uint8_t system_status;
  int16_t roll_cd;
  int16_t pitch_cd;
  int16_t yaw_cd;
  int32_t lat;
  int32_t lon;
  int16_t relative_alt_dm;
  uint16_t voltage_battery;
  int16_t current_battery;
  int8_t battery_remaining;
  uint16_t airspeed_cms;
  uint16_t groundspeed_cms;
  uint16_t heading;
  int16_t climb_cms;
  uint8_t throttle;
  uint16_t ekf_flags;
  GpsDataRingkas gps;
};

struct __attribute__((packed)) TelemetryBeaconPacket {
  PacketHeader hdr;
  uint32_t counter;
  uint8_t radio_packed;
  int8_t remote_snr_x2;
  uint8_t remote_rssi_q;
  uint8_t link_mode;
  uint16_t latency_x100;
  TelemetryMeta meta;
  PixhawkDataBeacon data;
};

struct __attribute__((packed)) MavlinkRawPacket {
  PacketHeader hdr;
  uint8_t len;
  uint8_t payload[RAW_MAVLINK_MAX];
};

struct __attribute__((packed)) CompactCommandBasePacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t kind;
};

struct __attribute__((packed)) CompactArmDisarmPacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t kind;
  uint8_t target_system;
  uint8_t target_component;
  uint8_t arm;
  int32_t param2_x1000;
};

struct __attribute__((packed)) CompactSetModePacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t kind;
  uint8_t target_system;
  uint8_t base_mode;
  uint32_t custom_mode;
};

struct __attribute__((packed)) CompactCommandLongPacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t kind;
  uint16_t command;
  uint8_t target_system;
  uint8_t target_component;
  uint8_t confirmation;
  int32_t p1_x1000;
  int32_t p2_x1000;
  int32_t p3_x1000;
  int32_t p4_x1000;
  int32_t p5_x1e7;
  int32_t p6_x1e7;
  int32_t p7_x1000;
};

#define COMPACT_CMD_MAX_LEN sizeof(CompactCommandLongPacket)

struct __attribute__((packed)) CompactCommandQueueItem {
  uint8_t len;
  uint16_t command;
  uint8_t payload[COMPACT_CMD_MAX_LEN];
};

struct __attribute__((packed)) CompactParamValue {
  float param_value;
  uint16_t param_count;
  uint16_t param_index;
  char param_id[16];
  uint8_t param_type;
};

struct __attribute__((packed)) ParamBulkPacket {
  PacketHeader hdr;
  uint8_t count;
  uint8_t sysid;
  uint8_t compid;
  uint8_t reserved;
  CompactParamValue rec[PARAM_BULK_MAX_RECORDS];
};

struct __attribute__((packed)) LinkAckPacket {
  PacketHeader hdr;
  uint32_t counter;
};

struct __attribute__((packed)) ConfigProposalPacket {
  PacketHeader hdr;
  uint32_t counter;
  uint32_t apply_counter;
  uint8_t current_sf;
  uint8_t current_tp;
  uint8_t next_sf;
  uint8_t next_tp;
  uint8_t next_profile;
};

struct __attribute__((packed)) ConfigAckPacket {
  PacketHeader hdr;
  uint32_t counter;
  uint32_t apply_counter;
  uint8_t accepted;
  uint8_t next_sf;
  uint8_t next_tp;
  uint8_t next_profile;
};

// ================= Utility Helper Functions (static inline) =================

static inline uint8_t packRadioParams78(uint8_t sf, uint8_t tp, uint8_t profile, uint8_t sf_min, uint8_t sf_max, uint8_t tp_min, uint8_t tp_max) {
  if (sf < sf_min) sf = sf_min;
  if (sf > sf_max) sf = sf_max;
  if (tp < tp_min) tp = tp_min;
  if (tp > tp_max) tp = tp_max;
  return (uint8_t)(((sf - sf_min) & 0x07) | (((tp - tp_min) & 0x0F) << 3) | ((profile & 0x01) << 7));
}

static inline uint8_t unpackSf78(uint8_t packed, uint8_t sf_min, uint8_t sf_max) {
  uint8_t sf = (uint8_t)((packed & 0x07) + sf_min);
  if (sf < sf_min) sf = sf_min;
  if (sf > sf_max) sf = sf_max;
  return sf;
}

static inline uint8_t unpackTp78(uint8_t packed, uint8_t tp_min, uint8_t tp_max) {
  uint8_t tp = (uint8_t)(((packed >> 3) & 0x0F) + tp_min);
  if (tp < tp_min) tp = tp_min;
  if (tp > tp_max) tp = tp_max;
  return tp;
}

static inline uint8_t unpackProfile78(uint8_t packed) {
  return (uint8_t)((packed >> 7) & 0x01);
}

static inline uint8_t loraRssiDbmToQ254(float rssiDbm) {
  if (rssiDbm <= -125.0f) return 1;
  if (rssiDbm >= -45.0f) return 254;
  float q = ((rssiDbm + 125.0f) * 253.0f / 80.0f) + 1.0f;
  if (q < 1.0f) q = 1.0f;
  if (q > 254.0f) q = 254.0f;
  return (uint8_t)(q + 0.5f);
}

static inline int8_t loraSnrDbToX2(float snrDb) {
  if (snrDb < -63.5f) snrDb = -63.5f;
  if (snrDb > 63.5f) snrDb = 63.5f;
  float x = snrDb * 2.0f;
  return (int8_t)(x >= 0.0f ? (x + 0.5f) : (x - 0.5f));
}

static inline float loraQ254ToRssiDbm(uint8_t q) {
  if (q <= 1) return -125.0f;
  if (q >= 254) return -45.0f;
  return (((float)q - 1.0f) * 80.0f / 253.0f) - 125.0f;
}

static inline int loraRfLinkMarginPercent(float rssiDbm, float snrDb) {
  int rssiPct;
  if (rssiDbm >= -95.0f) rssiPct = 100;
  else if (rssiDbm <= -125.0f) rssiPct = 0;
  else rssiPct = (int)(((rssiDbm + 125.0f) * 100.0f) / 30.0f);

  int snrPct;
  if (snrDb >= 6.0f) snrPct = 100;
  else if (snrDb <= -15.0f) snrPct = 0;
  else snrPct = (int)(((snrDb + 15.0f) * 100.0f) / 21.0f);

  int pct = (rssiPct * 55 + snrPct * 45) / 100;
  return (pct < 0) ? 0 : ((pct > 100) ? 100 : pct);
}

static inline float estimateTxCurrentMa(uint8_t tpDbm) {
  if (tpDbm <= 10) return 29.0f;
  if (tpDbm <= 12) return 45.0f;
  if (tpDbm <= 14) return 90.0f;
  return 120.0f;
}

static inline float estimateLoRaToA_ms(uint8_t sf, float bwHz, uint8_t crDen, uint16_t payloadBytes, float preambleSymbols) {
  uint8_t de = 0;
  if (sf >= 11 && bwHz <= 125000.0f) de = 1;
  float tsymMs = (powf(2.0f, sf) / bwHz) * 1000.0f;
  float preambleMs = (preambleSymbols + 4.25f) * tsymMs;
  float numerator = (8.0f * payloadBytes) - (4.0f * sf) + 28.0f + (16.0f * LORA_PHY_CRC_ENABLED) - (20.0f * LORA_IMPLICIT_HEADER);
  float denominator = 4.0f * (sf - (2.0f * de));
  float payloadSymbols = 8.0f;
  if (numerator > 0.0f && denominator > 0.0f) {
    payloadSymbols += ceilf(numerator / denominator) * crDen;
  }
  return preambleMs + (payloadSymbols * tsymMs);
}

static inline float estimatePacketEnergy_mJ(uint8_t tpDbm, float toaMs, float supplyVoltage) {
  float currentA = estimateTxCurrentMa(tpDbm) / 1000.0f;
  float timeS = toaMs / 1000.0f;
  return supplyVoltage * currentA * timeS * 1000.0f;
}

static inline float loraNominalBitrateKbps(uint8_t sf, float bwHz, uint8_t crDen) {
  return ((float)sf * bwHz * (4.0f / (float)crDen)) / (powf(2.0f, sf) * 1000.0f);
}

static inline bool isPacketCrcByte(size_t i) {
  return (i == offsetof(PacketHeader, crc) || i == (offsetof(PacketHeader, crc) + 1));
}

static inline bool isPacketAuthByte(size_t i) {
  size_t off = offsetof(PacketHeader, token);
  return (i >= off && i < off + sizeof(uint32_t));
}

static inline uint16_t computePacketCrc(const uint8_t *buf, size_t len) {
  if (len < sizeof(PacketHeader)) return 0;
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    uint8_t v = buf[i];
    if (isPacketCrcByte(i) || isPacketAuthByte(i)) v = 0;
    crc ^= ((uint16_t)v << 8);
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

static inline uint32_t computePacketAuthTag(const uint8_t *buf, size_t len) {
  uint32_t h = 2166136261UL ^ AUTH_TOKEN ^ SECURITY_KEY_MIX ^ ((uint32_t)NETWORK_ID << 8) ^ (uint32_t)PROTOCOL_VERSION;
  for (size_t i = 0; i < len; i++) {
    uint8_t v = buf[i];
    if (isPacketAuthByte(i)) v = 0;
    h ^= v;
    h *= 16777619UL;
    h ^= (h >> 13);
  }
  h ^= (uint32_t)len * 0x9E3779B9UL;
  h ^= (h >> 16);
  h *= 0x7FEB352DUL;
  h ^= (h >> 15);
  h *= 0x846CA68BUL;
  h ^= (h >> 16);
  if (h == 0 || h == AUTH_TOKEN) h ^= 0xA5A55A5AUL;
  return h;
}

static inline void initHeader(PacketHeader &hdr, uint8_t type) {
  hdr.type = type;
  hdr.protocol = PROTOCOL_VERSION;
  hdr.network_id = NETWORK_ID;
  hdr.crc = 0;
  hdr.token = 0;
}

static inline void finalizePacketCrc(void *packet, size_t len) {
  PacketHeader *hdr = (PacketHeader *)packet;
  hdr->token = 0;
  hdr->crc = 0;
  hdr->crc = computePacketCrc((uint8_t *)packet, len);
  hdr->token = computePacketAuthTag((uint8_t *)packet, len);
}

static inline bool validatePacketInternal(const uint8_t *buf, size_t len, uint32_t &lenDrop, uint32_t &protoDrop, uint32_t &crcDrop) {
  if (len < sizeof(PacketHeader)) { lenDrop++; return false; }
  const PacketHeader *hdr = (const PacketHeader *)buf;
  if (hdr->protocol != PROTOCOL_VERSION || hdr->network_id != NETWORK_ID) {
    protoDrop++; return false;
  }
  uint16_t calc = computePacketCrc(buf, len);
  if (calc != hdr->crc) { crcDrop++; return false; }
  uint32_t tag = computePacketAuthTag(buf, len);
  if (tag != hdr->token) { protoDrop++; return false; }
  return true;
}

// ================= Compile-time Assertions =================
static_assert(sizeof(TelemetryBeaconPacket) == 78, "Size mismatch: TelemetryBeaconPacket size must be 78 bytes");
static_assert(sizeof(PixhawkDataBeacon) == 49, "Size mismatch: PixhawkDataBeacon size must be 49 bytes");
static_assert(sizeof(TelemetryMeta) == 9, "Size mismatch: TelemetryMeta size must be 9 bytes");
static_assert(sizeof(GpsDataRingkas) == 8, "Size mismatch: GpsDataRingkas size must be 8 bytes");

#endif
