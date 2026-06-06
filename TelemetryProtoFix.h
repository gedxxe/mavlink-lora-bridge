#include "SharedConfig.h"
#ifndef TELEMETRY_PROTO_FIX_H
#define TELEMETRY_PROTO_FIX_H
#include <stdint.h>
#include <stddef.h>

// Consolidated struct definitions from GCS and UAV
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

struct __attribute__((packed)) GpsDataRingkas {
  uint8_t fix_type;
  uint8_t satellites_visible;
  int16_t alt_dm;      // UAV mengirim decimeter; GCS mengembalikan ke mm.
  uint16_t eph;
  uint16_t epv;
};

struct __attribute__((packed)) TelemetryMeta {
  uint8_t cnt_ack;
  uint16_t success_streak;
  uint16_t fail_streak;
  uint32_t telem_tx_energy_mJ_x100;
};

struct __attribute__((packed)) PixhawkDataFull {
  uint32_t valid_flags;
  uint8_t system_id;
  uint8_t component_id;
  uint8_t mav_type;
  uint8_t autopilot;
  uint8_t base_mode;
  uint32_t custom_mode;
  uint8_t system_status;
  float roll;
  float pitch;
  float yaw;
  float rollspeed;
  float pitchspeed;
  float yawspeed;
  int32_t lat;
  int32_t lon;
  int32_t alt_mm;
  int32_t relative_alt_mm;
  int16_t vx;
  int16_t vy;
  int16_t vz;
  uint16_t hdg;
  float airspeed;
  float groundspeed;
  int16_t heading;
  uint16_t throttle;
  float climb;
  uint16_t voltage_battery;
  int16_t current_battery;
  int8_t battery_remaining;
  uint16_t ekf_flags;
  GpsRawData gps;
};

struct __attribute__((packed)) PixhawkDataReduced {
  uint32_t valid_flags;
  uint8_t system_id;
  uint8_t component_id;
  uint8_t mav_type;
  uint8_t autopilot;
  uint8_t base_mode;
  uint32_t custom_mode;
  uint8_t system_status;
  float roll;
  float pitch;
  float yaw;
  int32_t lat;
  int32_t lon;
  int32_t relative_alt_mm;
  int16_t heading;
  float groundspeed;
  float climb;
  uint16_t voltage_battery;
  int8_t battery_remaining;
  uint16_t ekf_flags;
  GpsRawData gps;
};

struct __attribute__((packed)) PixhawkDataMinimal {
  uint32_t valid_flags;
  uint8_t system_id;
  uint8_t component_id;
  uint8_t base_mode;
  uint32_t custom_mode;
  uint8_t system_status;
  float roll;
  float pitch;
  float yaw;
  int32_t lat;
  int32_t lon;
  int32_t relative_alt_mm;
  uint16_t voltage_battery;
  int8_t battery_remaining;
  uint16_t heading;
  uint16_t groundspeed_cms;
  int16_t climb_cms;
  GpsRawData gps;
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
  uint16_t heading;       // centi-degree.
  int16_t climb_cms;
  uint8_t throttle;       // persen 0..100.

  uint16_t ekf_flags;
  GpsDataRingkas gps;
};

struct __attribute__((packed)) TelemetryBeaconPacket {
  PacketHeader hdr;
  uint32_t counter;

  // 3 byte radio/status ringkas:
  // - radio_packed menyimpan SF, TP, profile dalam 1 byte.
  // - remote_snr_x2 dan remote_rssi_q adalah kualitas downlink GCS->UAV yang diukur nyata oleh UAV.
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

#define COMPACT_CMD_MAX_LEN sizeof(struct CompactCommandLongPacket)

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

struct __attribute__((packed)) GpsRawDataFull {
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

#endif
