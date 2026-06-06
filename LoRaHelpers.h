#ifndef LORA_HELPERS_H
#define LORA_HELPERS_H

#include <stdint.h>
#include <math.h>

// Shared LoRa physical calculations
static inline float estimateLoRaToA_ms(uint8_t sf, float bwHz, uint8_t crDen, uint16_t payloadBytes, bool crcEnabled = true, bool implicitHeader = false, float preambleSymbols = 8.0f) {
  uint8_t de = 0;
  if (sf >= 11 && bwHz <= 125000.0f) de = 1;
  float tsymMs = (powf(2.0f, sf) / bwHz) * 1000.0f;
  float preambleMs = (preambleSymbols + 4.25f) * tsymMs;
  float numerator = (8.0f * payloadBytes) - (4.0f * sf) + 28.0f + (16.0f * crcEnabled) - (20.0f * implicitHeader);
  float denominator = 4.0f * (sf - (2.0f * de));
  float payloadSymbols = 8.0f;
  if (numerator > 0.0f && denominator > 0.0f) payloadSymbols += ceilf(numerator / denominator) * crDen;
  return preambleMs + (payloadSymbols * tsymMs);
}

static inline float loraNominalBitrateKbps(uint8_t sf, float bwHz, uint8_t crDen) {
  return ((float)sf * bwHz * (4.0f / (float)crDen)) / (powf(2.0f, sf) * 1000.0f);
}

static inline float estimateTxCurrentMa(uint8_t tpDbm) {
  if (tpDbm <= 10) return 29.0f;
  if (tpDbm <= 12) return 45.0f;
  if (tpDbm <= 14) return 90.0f;
  return 120.0f;
}

static inline float estimatePacketEnergy_mJ(uint8_t tpDbm, float toaMs, float supplyVoltage = 3.30f) {
  float currentA = estimateTxCurrentMa(tpDbm) / 1000.0f;
  float timeS = toaMs / 1000.0f;
  return supplyVoltage * currentA * timeS * 1000.0f;
}

static inline uint8_t loraRssiDbmToQ254(float rssiDbm) {
  if (rssiDbm <= -125.0f) return 1;
  if (rssiDbm >= -45.0f) return 254;
  float q = ((rssiDbm + 125.0f) * 253.0f / 80.0f) + 1.0f;
  if (q < 1.0f) q = 1.0f;
  if (q > 254.0f) q = 254.0f;
  return (uint8_t)(q + 0.5f);
}

static inline float loraQ254ToRssiDbm(uint8_t q) {
  if (q <= 1) return -125.0f;
  if (q >= 254) return -45.0f;
  return (((float)q - 1.0f) * 80.0f / 253.0f) - 125.0f;
}

static inline int8_t loraSnrDbToX2(float snrDb) {
  if (snrDb < -63.5f) snrDb = -63.5f;
  if (snrDb > 63.5f) snrDb = 63.5f;
  float x = snrDb * 2.0f;
  return (int8_t)(x >= 0.0f ? (x + 0.5f) : (x - 0.5f));
}

#endif // LORA_HELPERS_H
