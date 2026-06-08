// =============================================================================
// TelemetryProtoFix.h
// Shared OTA protocol definitions for the MAVLink-LoRa telemetry bridge.
//
// RULES FOR THIS FILE:
//  - All functions MUST be `static inline` to avoid multiple-definition linker
//    errors when both GCS and UAV compilation units include this header.
//  - DO NOT reference any global variables from sketch files here.
//  - DO NOT add Arduino-specific includes (Serial, millis, etc.).
//  - All packed structs are OTA wire format — never change field order/size.
// =============================================================================
#ifndef TELEMETRY_PROTO_FIX_H
#define TELEMETRY_PROTO_FIX_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Forward declarations (required before Arduino IDE auto-prototype injection)
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Protocol constants (NEVER change — must match on both ends of the LoRa link)
// ---------------------------------------------------------------------------
#define PROTOCOL_VERSION            0x04
#define NETWORK_ID                  0x2244
#define AUTH_TOKEN                  0xA56C93D1UL   // Pre-shared secret for per-packet auth-tag
#define SECURITY_KEY_MIX            0x3D7F21B9UL
#define SECURITY_ANTI_REPLAY_ENABLE 1
#define SECURITY_MAX_COUNTER_GAP    5000UL
#define SECURITY_REBOOT_GRACE_MS    15000UL

// ---------------------------------------------------------------------------
// Packet type identifiers (1-byte type field in PacketHeader)
// ---------------------------------------------------------------------------
#define PKT_LINK_ACK          0xA5
#define PKT_MAVLINK_RAW       0x4D
#define PKT_CMD_COMPACT       0x43
#define PKT_PARAM_BULK        0x50
#define PKT_TELEM_BEACON      0x57
#define PKT_PARAM_DEBUG       0x58  // Best-effort debug snapshot; never required for telemetry/param sync
#define PKT_CONFIG_PROPOSE    0xC0
#define PKT_CONFIG_ACK        0xC1

// ---------------------------------------------------------------------------
// Shared capacity / protocol constants
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// valid_flags bits (uint8_t, bit-field for PixhawkDataBeacon.valid_flags)
// ---------------------------------------------------------------------------
#define VALID_HEARTBEAT   (1U << 0)
#define VALID_ATTITUDE    (1U << 1)
#define VALID_GLOBAL_POS  (1U << 2)
#define VALID_VFR_HUD     (1U << 3)
#define VALID_SYS_STATUS  (1U << 4)
#define VALID_EKF         (1U << 5)
#define VALID_GPS_RAW     (1U << 6)
#define VALID_LOCAL_VEL   (1U << 7)   // vx_cms / vy_cms / vz_cms valid

// valid_flags2 bits (uint8_t, secondary validity flags)
#define VALID2_VIBRATION    (1U << 0)   // vibe_x/y/z_x100, accel_clip valid
#define VALID2_RC_CHANNELS  (1U << 1)   // rc_ch_pct[16] valid

// ---------------------------------------------------------------------------
// RC channel encoding helpers
// RC channels arrive from MAVLink as raw PWM (typically 1000–2000 µs).
// We encode as 0–100 % unsigned byte to save 8 bytes vs uint16_t[16].
// ---------------------------------------------------------------------------
#define RC_RAW_MIN   1000U
#define RC_RAW_MAX   2000U
#define RC_RAW_RANGE  (RC_RAW_MAX - RC_RAW_MIN)   // 1000

static inline uint8_t rcRawToPct(uint16_t raw) {
  if (raw == 65535U) return 255;
  if (raw <= RC_RAW_MIN) return 0;
  if (raw >= RC_RAW_MAX) return 100;
  return (uint8_t)(((uint32_t)(raw - RC_RAW_MIN) * 100U) / RC_RAW_RANGE);
}

static inline uint16_t rcPctToRaw(uint8_t pct) {
  if (pct == 255) return 65535U;
  return (uint16_t)(RC_RAW_MIN + ((uint32_t)pct * RC_RAW_RANGE) / 100U);
}

// =============================================================================
// OTA Struct Definitions  —  NEVER reorder or resize fields
// All structs use __attribute__((packed)) to prevent padding.
// =============================================================================

// ---------------------------------------------------------------------------
// 10-byte packet header (present in every packet type)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) PacketHeader {
  uint8_t  type;        // PKT_* identifier
  uint8_t  protocol;    // PROTOCOL_VERSION
  uint16_t network_id;  // NETWORK_ID
  uint16_t crc;         // CRC-16/CCITT-FALSE over whole packet (crc/token zeroed)
  uint32_t token;       // Lightweight keyed auth-tag (depends on crc + secret)
};

// ---------------------------------------------------------------------------
// Full GPS struct — used only in UAV-side PixhawkDataFull (not transmitted)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) GpsRawData {
  uint64_t time_usec;
  uint8_t  fix_type;
  int32_t  lat;
  int32_t  lon;
  int32_t  alt_mm;
  uint16_t eph;
  uint16_t epv;
  uint16_t vel;
  uint16_t cog;
  uint8_t  satellites_visible;
};
typedef GpsRawData GpsRawDataFull;  // UAV compatibility alias

// ---------------------------------------------------------------------------
// Compact GPS — 8 bytes, transmitted inside PixhawkDataBeacon
// ---------------------------------------------------------------------------
struct __attribute__((packed)) GpsDataRingkas {
  uint8_t  fix_type;
  uint8_t  satellites_visible;
  int16_t  alt_dm;       // GPS altitude in decimeters (GCS converts back to mm)
  uint16_t eph;
  uint16_t epv;
};

// ---------------------------------------------------------------------------
// Telemetry link meta — 9 bytes
//
// Keep the high-rate beacon compact. Debug counters are deliberately moved to
// PKT_PARAM_DEBUG so SF7 Turbo is not slowed by diagnostic fields in every
// telemetry packet. This restores TelemetryBeaconPacket to 109 bytes OTA.
// ---------------------------------------------------------------------------
struct __attribute__((packed)) TelemetryMeta {
  uint8_t  cnt_ack;                  // Successful ACKs in last 10 beacon windows
  uint16_t success_streak;
  uint16_t fail_streak;
  uint32_t telem_tx_energy_mJ_x100; // Cumulative TX energy × 100 (mJ)
};

// ---------------------------------------------------------------------------
// Main sensor payload — 80 bytes — transmitted in every TelemetryBeaconPacket
//
// Field notes:
//   valid_flags   — bits 0-7 per VALID_* defines above
//   valid_flags2  — bits per VALID2_* defines above (NEW)
//   heading       — centi-degrees (matches Mission Planner convention)
//   vx/vy/vz_cms  — NED frame, cm/s (NEW)
//   vibe_*_x100   — vibration in m/s², stored as int16_t × 100 (NEW)
//   accel_clip    — sum of clipping_0+1+2 from MAVLink VIBRATION msg (NEW)
//   rc_ch_pct     — RC channels 1-16 encoded as 0-100% (NEW, 16 bytes)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) PixhawkDataBeacon {
  // ── Core status ─────────────────────────────────
  uint8_t  valid_flags;        // Existing validity bits (HEARTBEAT..LOCAL_VEL)
  uint8_t  valid_flags2;       // Extended validity bits (VIBRATION, RC_CHANNELS) [NEW]
  uint8_t  system_id;
  uint8_t  component_id;
  uint8_t  base_mode;
  uint32_t custom_mode;
  uint8_t  system_status;

  // ── Attitude ────────────────────────────────────
  int16_t  roll_cd;            // centi-degrees
  int16_t  pitch_cd;
  int16_t  yaw_cd;

  // ── Position ────────────────────────────────────
  int32_t  lat;                // degrees × 1e7
  int32_t  lon;
  int16_t  relative_alt_dm;   // decimeters

  // ── Battery ─────────────────────────────────────
  uint16_t voltage_battery;   // mV
  int16_t  current_battery;   // cA (centipamps)
  int8_t   battery_remaining; // %

  // ── Airdata ─────────────────────────────────────
  uint16_t airspeed_cms;
  uint16_t groundspeed_cms;
  uint16_t heading;            // centi-degrees
  int16_t  climb_cms;
  uint8_t  throttle;           // 0-100%

  // ── EKF + GPS ───────────────────────────────────
  uint16_t     ekf_flags;
  GpsDataRingkas gps;          // 8 bytes

  // ── Velocity (NED) [NEW] ─────────────────────────
  int16_t  vx_cms;             // North velocity, cm/s
  int16_t  vy_cms;             // East  velocity, cm/s
  int16_t  vz_cms;             // Down  velocity, cm/s

  // ── Vibration [NEW] ─────────────────────────────
  int16_t  vibe_x_x100;       // X vibration × 100 (m/s²)
  int16_t  vibe_y_x100;       // Y vibration × 100
  int16_t  vibe_z_x100;       // Z vibration × 100
  uint16_t accel_clip;         // Total accel clipper count

  // ── RC Channels 1-16 [NEW] ──────────────────────
  uint8_t  rc_ch_pct[16];     // Channels 1-16, each 0-100%
};

// ---------------------------------------------------------------------------
// Main beacon packet — 109 bytes OTA
// Layout: PacketHeader(10) + counter(4) + radio(1) + snr(1) + rssi(1)
//         + mode(1) + latency(2) + TelemetryMeta(9) + PixhawkDataBeacon(80)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) TelemetryBeaconPacket {
  PacketHeader     hdr;
  uint32_t         counter;
  uint8_t          radio_packed;   // SF, TP, profile packed into 1 byte
  int8_t           remote_snr_x2;  // UAV-measured GCS-downlink SNR × 2
  uint8_t          remote_rssi_q;  // UAV-measured GCS-downlink RSSI quantised 1-254
  uint8_t          link_mode;      // LINK_MODE_* constant
  uint16_t         latency_x100;   // Half-RTT × 100 ms
  TelemetryMeta    meta;
  PixhawkDataBeacon data;
};

// Best-effort debug snapshot, sent rarely (20 s by default) so diagnostics do
// not inflate every beacon. The GCS stores the latest snapshot and prints it in
// MetricsSerial next to normal telemetry rows. No ACK/retry is required.
struct __attribute__((packed)) ParamDebugPacket {
  PacketHeader hdr;
  uint16_t dbg_async_start;
  uint16_t dbg_async_req_tx;
  uint16_t dbg_async_retry;
  uint16_t dbg_param_bulk_queued;
  uint16_t dbg_param_bulk_tx;
  uint16_t dbg_param_bulk_ack;
  uint16_t dbg_param_bulk_fail;
  uint16_t dbg_param_value_drop;
  uint16_t dbg_full_param_blocked;
  uint16_t dbg_async_index;
  uint16_t dbg_async_total;
  uint8_t  dbg_param_state;
};

// ---------------------------------------------------------------------------
// Raw MAVLink relay packet
// ---------------------------------------------------------------------------
struct __attribute__((packed)) MavlinkRawPacket {
  PacketHeader hdr;
  uint8_t len;
  uint8_t payload[RAW_MAVLINK_MAX];
};

// ---------------------------------------------------------------------------
// Compact flight command packets (GCS → UAV downlink)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) CompactCommandBasePacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t  kind;
};

struct __attribute__((packed)) CompactArmDisarmPacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t  kind;
  uint8_t  target_system;
  uint8_t  target_component;
  uint8_t  arm;
  int32_t  param2_x1000;
};

struct __attribute__((packed)) CompactSetModePacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t  kind;
  uint8_t  target_system;
  uint8_t  base_mode;
  uint32_t custom_mode;
};

struct __attribute__((packed)) CompactCommandLongPacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t  kind;
  uint16_t command;
  uint8_t  target_system;
  uint8_t  target_component;
  uint8_t  confirmation;
  int32_t  p1_x1000;
  int32_t  p2_x1000;
  int32_t  p3_x1000;
  int32_t  p4_x1000;
  int32_t  p5_x1e7;
  int32_t  p6_x1e7;
  int32_t  p7_x1000;
};

#define COMPACT_CMD_MAX_LEN sizeof(CompactCommandLongPacket)

struct __attribute__((packed)) CompactCommandQueueItem {
  uint8_t  len;
  uint16_t command;
  uint8_t  payload[COMPACT_CMD_MAX_LEN];
};

// ---------------------------------------------------------------------------
// Parameter bulk transfer
// ---------------------------------------------------------------------------
struct __attribute__((packed)) CompactParamValue {
  float    param_value;
  uint16_t param_count;
  uint16_t param_index;
  char     param_id[16];
  uint8_t  param_type;
};

struct __attribute__((packed)) ParamBulkPacket {
  PacketHeader   hdr;
  uint8_t        count;
  uint8_t        sysid;
  uint8_t        compid;
  uint8_t        reserved;
  CompactParamValue rec[PARAM_BULK_MAX_RECORDS];
};

// ---------------------------------------------------------------------------
// Link acknowledgement and config negotiation packets
// ---------------------------------------------------------------------------
struct __attribute__((packed)) LinkAckPacket {
  PacketHeader hdr;
  uint32_t counter;
};

struct __attribute__((packed)) ConfigProposalPacket {
  PacketHeader hdr;
  uint32_t counter;
  uint32_t apply_counter;
  uint8_t  current_sf;
  uint8_t  current_tp;
  uint8_t  next_sf;
  uint8_t  next_tp;
  uint8_t  next_profile;
};

struct __attribute__((packed)) ConfigAckPacket {
  PacketHeader hdr;
  uint32_t counter;
  uint32_t apply_counter;
  uint8_t  accepted;
  uint8_t  next_sf;
  uint8_t  next_tp;
  uint8_t  next_profile;
};

// =============================================================================
// Pure utility helpers (static inline, no global-variable dependencies)
// =============================================================================

// ---------------------------------------------------------------------------
// Radio parameter packing (SF + TP + profile → 1 byte)
// Bits [2:0] = SF offset from sf_min (max range 0-7 → SF7-SF14)
// Bits [6:3] = TP offset from tp_min (max range 0-15 → TP10-TP25)
// Bit  [7]   = profile (0 or 1)
// ---------------------------------------------------------------------------
static inline uint8_t packRadioParams78(uint8_t sf, uint8_t tp, uint8_t profile,
                                        uint8_t sf_min, uint8_t sf_max,
                                        uint8_t tp_min, uint8_t tp_max) {
  if (sf < sf_min) sf = sf_min;  if (sf > sf_max) sf = sf_max;
  if (tp < tp_min) tp = tp_min;  if (tp > tp_max) tp = tp_max;
  return (uint8_t)(((sf - sf_min) & 0x07) |
                   (((tp - tp_min) & 0x0F) << 3) |
                   ((profile & 0x01) << 7));
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

// ---------------------------------------------------------------------------
// LoRa signal metric conversions
// ---------------------------------------------------------------------------

// RSSI → quantised 1-254 (0 = invalid/no measurement)
static inline uint8_t loraRssiDbmToQ254(float rssiDbm) {
  if (rssiDbm <= -125.0f) return 1;
  if (rssiDbm >= -45.0f)  return 254;
  float q = ((rssiDbm + 125.0f) * 253.0f / 80.0f) + 1.0f;
  if (q < 1.0f) q = 1.0f;
  if (q > 254.0f) q = 254.0f;
  return (uint8_t)(q + 0.5f);
}

// SNR (dB) → int8_t × 2  (range ±63.5 dB)
static inline int8_t loraSnrDbToX2(float snrDb) {
  if (snrDb < -63.5f) snrDb = -63.5f;
  if (snrDb >  63.5f) snrDb =  63.5f;
  float x = snrDb * 2.0f;
  return (int8_t)(x >= 0.0f ? (x + 0.5f) : (x - 0.5f));
}

// Reverse: quantised → RSSI dBm
static inline float loraQ254ToRssiDbm(uint8_t q) {
  if (q <= 1)   return -125.0f;
  if (q >= 254) return  -45.0f;
  return (((float)q - 1.0f) * 80.0f / 253.0f) - 125.0f;
}

// Combined RF link margin (0-100 %)  – used for RADIO_STATUS.rssi
static inline int loraRfLinkMarginPercent(float rssiDbm, float snrDb) {
  int rssiPct;
  if      (rssiDbm >= -95.0f)  rssiPct = 100;
  else if (rssiDbm <= -125.0f) rssiPct = 0;
  else rssiPct = (int)(((rssiDbm + 125.0f) * 100.0f) / 30.0f);

  int snrPct;
  if      (snrDb >=  6.0f) snrPct = 100;
  else if (snrDb <= -15.0f) snrPct = 0;
  else snrPct = (int)(((snrDb + 15.0f) * 100.0f) / 21.0f);

  int pct = (rssiPct * 55 + snrPct * 45) / 100;
  return (pct < 0) ? 0 : ((pct > 100) ? 100 : pct);
}

// ---------------------------------------------------------------------------
// Power / airtime estimators
// ---------------------------------------------------------------------------

static inline float estimateTxCurrentMa(uint8_t tpDbm) {
  if (tpDbm <= 10) return  29.0f;
  if (tpDbm <= 12) return  45.0f;
  if (tpDbm <= 14) return  90.0f;
  return 120.0f;
}

// preambleSymbols is passed by caller (GCS=8, UAV=12) so the header remains
// free of any sketch-specific constants.
static inline float estimateLoRaToA_ms(uint8_t sf, float bwHz, uint8_t crDen,
                                        uint16_t payloadBytes,
                                        float preambleSymbols) {
  uint8_t de = (sf >= 11 && bwHz <= 125000.0f) ? 1 : 0;
  float tsymMs      = (powf(2.0f, sf) / bwHz) * 1000.0f;
  float preambleMs  = (preambleSymbols + 4.25f) * tsymMs;
  float numerator   = (8.0f * payloadBytes) - (4.0f * sf) + 28.0f
                      + (16.0f * LORA_PHY_CRC_ENABLED)
                      - (20.0f * LORA_IMPLICIT_HEADER);
  float denominator = 4.0f * ((float)sf - (2.0f * de));
  float paySymbols  = 8.0f;
  if (numerator > 0.0f && denominator > 0.0f)
    paySymbols += ceilf(numerator / denominator) * (float)crDen;
  return preambleMs + (paySymbols * tsymMs);
}

static inline float estimatePacketEnergy_mJ(uint8_t tpDbm, float toaMs,
                                             float supplyVoltage) {
  float currentA = estimateTxCurrentMa(tpDbm) / 1000.0f;
  float timeS    = toaMs / 1000.0f;
  return supplyVoltage * currentA * timeS * 1000.0f;
}

static inline float loraNominalBitrateKbps(uint8_t sf, float bwHz, uint8_t crDen) {
  return ((float)sf * bwHz * (4.0f / (float)crDen)) / (powf(2.0f, sf) * 1000.0f);
}

// ---------------------------------------------------------------------------
// Packet CRC / authentication
// ---------------------------------------------------------------------------

static inline bool isPacketCrcByte(size_t i) {
  return (i == offsetof(PacketHeader, crc) ||
          i == (offsetof(PacketHeader, crc) + 1));
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
      else              crc <<= 1;
    }
  }
  return crc;
}

static inline uint32_t computePacketAuthTag(const uint8_t *buf, size_t len) {
  uint32_t h = 2166136261UL ^ AUTH_TOKEN ^ SECURITY_KEY_MIX
               ^ ((uint32_t)NETWORK_ID << 8)
               ^ (uint32_t)PROTOCOL_VERSION;
  for (size_t i = 0; i < len; i++) {
    uint8_t v = buf[i];
    if (isPacketAuthByte(i)) v = 0;
    h ^= v;
    h *= 16777619UL;
    h ^= (h >> 13);
  }
  h ^= (uint32_t)len * 0x9E3779B9UL;
  h ^= (h >> 16); h *= 0x7FEB352DUL;
  h ^= (h >> 15); h *= 0x846CA68BUL;
  h ^= (h >> 16);
  if (h == 0 || h == AUTH_TOKEN) h ^= 0xA5A55A5AUL;
  return h;
}

static inline void initHeader(PacketHeader &hdr, uint8_t type) {
  hdr.type       = type;
  hdr.protocol   = PROTOCOL_VERSION;
  hdr.network_id = NETWORK_ID;
  hdr.crc        = 0;
  hdr.token      = 0;
}

static inline void finalizePacketCrc(void *packet, size_t len) {
  PacketHeader *hdr = (PacketHeader *)packet;
  hdr->token = 0;
  hdr->crc   = 0;
  hdr->crc   = computePacketCrc((uint8_t *)packet, len);
  hdr->token = computePacketAuthTag((uint8_t *)packet, len);
}

// Stateless validator — counters passed by reference so no global dependencies.
static inline bool validatePacketInternal(const uint8_t *buf, size_t len,
                                           uint32_t &lenDrop,
                                           uint32_t &protoDrop,
                                           uint32_t &crcDrop) {
  if (len < sizeof(PacketHeader)) { lenDrop++;   return false; }
  const PacketHeader *hdr = (const PacketHeader *)buf;
  if (hdr->protocol != PROTOCOL_VERSION || hdr->network_id != NETWORK_ID) {
    protoDrop++; return false;
  }
  if (computePacketCrc(buf, len) != hdr->crc) { crcDrop++;    return false; }
  if (computePacketAuthTag(buf, len) != hdr->token) { protoDrop++; return false; }
  return true;
}

// =============================================================================
// Compile-time struct size assertions — build fails if OTA layout drifts
// =============================================================================
static_assert(sizeof(PacketHeader)        == 10,  "PacketHeader must be 10 bytes");
static_assert(sizeof(GpsDataRingkas)      == 8,   "GpsDataRingkas must be 8 bytes");
static_assert(sizeof(TelemetryMeta)       == 9,   "TelemetryMeta must be 9 bytes in compact beacon build");
static_assert(sizeof(PixhawkDataBeacon)   == 80,  "PixhawkDataBeacon must be 80 bytes");
static_assert(sizeof(TelemetryBeaconPacket) == 109, "TelemetryBeaconPacket must be 109 bytes in compact beacon build");
static_assert(sizeof(ParamDebugPacket)     == 33,  "ParamDebugPacket must be 33 bytes");

#endif // TELEMETRY_PROTO_FIX_H
