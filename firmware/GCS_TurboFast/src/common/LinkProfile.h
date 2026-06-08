#ifndef LINK_PROFILE_H
#define LINK_PROFILE_H

#include <stdint.h>

// Central operating policy for the LoRa-MAVLink bridge.
// SF7 = turbo/setup, SF8 = normal, SF9 = long range. SF10-SF12 are recovery
// monitoring modes and should not carry full Mission Planner setup traffic.
#define LINK_SF_TURBO       7
#define LINK_SF_NORMAL      8
#define LINK_SF_LONG_RANGE  9
#define LINK_SF_SETUP_MAX   9
#define LINK_SF_MONITORING_MIN 10

#define LINK_MODE_NORMAL       0
#define LINK_MODE_PARAM_SYNC   1
#define LINK_MODE_CALIBRATION  2
#define LINK_MODE_MISSION      3
#define LINK_MODE_FAILSAFE     4
#define LINK_MODE_COMMAND      5

static inline uint8_t linkClampSf(uint8_t sf) {
  if (sf < 7) return 7;
  if (sf > 12) return 12;
  return sf;
}

static inline bool linkSfSupportsSetup(uint8_t sf) {
  return linkClampSf(sf) <= LINK_SF_SETUP_MAX;
}

static inline bool linkSfAllowsParamGet(uint8_t sf) {
  return linkSfSupportsSetup(sf);
}

static inline bool linkSfIsMonitoringOnly(uint8_t sf) {
  return linkClampSf(sf) >= LINK_SF_MONITORING_MIN;
}

static inline unsigned long linkNormalTelemetryIntervalMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 300UL;
    case LINK_SF_NORMAL:     return 450UL;
    case LINK_SF_LONG_RANGE: return 700UL;
    case 10:                 return 1000UL;
    case 11:                 return 1500UL;
    default:                 return 2200UL;
  }
}

static inline unsigned long linkParamSyncTelemetryIntervalMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 1200UL;
    case LINK_SF_NORMAL:     return 1600UL;
    case LINK_SF_LONG_RANGE: return 2200UL;
    case 10:                 return 3200UL;
    case 11:                 return 4500UL;
    default:                 return 6000UL;
  }
}

static inline unsigned long linkParamNoValueTimeoutMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 14000UL;
    case LINK_SF_NORMAL:     return 18000UL;
    case LINK_SF_LONG_RANGE: return 24000UL;
    case 10:                 return 16000UL;
    case 11:                 return 10000UL;
    default:                 return 8000UL;
  }
}

static inline unsigned long linkParamIdleTimeoutMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 9000UL;
    case LINK_SF_NORMAL:     return 11000UL;
    case LINK_SF_LONG_RANGE: return 14000UL;
    case 10:                 return 5000UL;
    default:                 return 8000UL;
  }
}

static inline unsigned long linkParamStallTimeoutMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 12000UL;
    case LINK_SF_NORMAL:     return 18000UL;
    case LINK_SF_LONG_RANGE: return 26000UL;
    case 10:                 return 16000UL;
    case 11:                 return 14000UL;
    default:                 return 12000UL;
  }
}

static inline unsigned long linkRawAckTimeoutMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 320UL;
    case LINK_SF_NORMAL:     return 460UL;
    case LINK_SF_LONG_RANGE: return 680UL;
    case 10:                 return 1050UL;
    case 11:                 return 1500UL;
    default:                 return 2100UL;
  }
}

static inline unsigned long linkParamBulkAckTimeoutMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 180UL;
    case LINK_SF_NORMAL:     return 260UL;
    case LINK_SF_LONG_RANGE: return 380UL;
    case 10:                 return 850UL;
    case 11:                 return 1300UL;
    default:                 return 1900UL;
  }
}

static inline unsigned long linkTelemetryAckTimeoutMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 120UL;
    case LINK_SF_NORMAL:     return 180UL;
    case LINK_SF_LONG_RANGE: return 260UL;
    default:                 return linkRawAckTimeoutMs(sf);
  }
}

static inline unsigned long linkParamBulkFillGraceMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 80UL;
    case LINK_SF_NORMAL:     return 130UL;
    case LINK_SF_LONG_RANGE: return 190UL;
    default:                 return 250UL;
  }
}

static inline unsigned long linkParamRequestRetryMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 650UL;
    case LINK_SF_NORMAL:     return 900UL;
    case LINK_SF_LONG_RANGE: return 1200UL;
    default:                 return 1500UL;
  }
}

static inline uint8_t linkParamBulkFillWindow(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 5;
    case LINK_SF_NORMAL:     return 4;
    case LINK_SF_LONG_RANGE: return 3;
    default:                 return 2;
  }
}

static inline uint8_t linkParamBulkRecords(uint8_t sf, uint8_t maxRecords) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return maxRecords;
    case LINK_SF_NORMAL:     return (maxRecords >= 8) ? 8 : maxRecords;
    case LINK_SF_LONG_RANGE: return (maxRecords >= 6) ? 6 : maxRecords;
    default:                 return 0;
  }
}

static inline unsigned long linkGcsRxTimeoutMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 220UL;
    case LINK_SF_NORMAL:     return 320UL;
    case LINK_SF_LONG_RANGE: return 460UL;
    case 10:                 return 700UL;
    case 11:                 return 1000UL;
    default:                 return 1350UL;
  }
}

static inline unsigned long linkGcsIdleScanAfterMs(uint8_t sf) {
  switch (linkClampSf(sf)) {
    case LINK_SF_TURBO:      return 1400UL;
    case LINK_SF_NORMAL:     return 2200UL;
    case LINK_SF_LONG_RANGE: return 3200UL;
    case 10:                 return 4800UL;
    case 11:                 return 6500UL;
    default:                 return 8500UL;
  }
}

static inline uint8_t linkWeightedScanSf(uint8_t step) {
  static const uint8_t sequence[] = {
    7, 8, 7, 9, 8, 7, 9, 8,
    7, 10, 9, 8, 7, 11, 9, 12
  };
  return sequence[step % (sizeof(sequence) / sizeof(sequence[0]))];
}

#endif // LINK_PROFILE_H
