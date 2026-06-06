#ifndef UAV_RADIO_HELPERS_H
#define UAV_RADIO_HELPERS_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "TelemetryProtoFix.h"

// Extern references to variables in main UAV sketch
extern SX1278 radio;
extern uint8_t currentSF;
extern uint8_t currentTP;
extern uint8_t currentProfile;
extern uint32_t loraInitRetrySuccessCount;
extern unsigned long lastRadioRecoveryMs;
extern uint32_t radioSoftRecoveryCount;
extern bool paramSyncActive;
extern int lowCount;

#ifndef LORA_BW_KHZ
#define LORA_BW_KHZ 250.0f
#endif
#ifndef LORA_CR_DEN
#define LORA_CR_DEN 8
#endif
#ifndef FREQ_MHZ
#define FREQ_MHZ 433.0f
#endif
#ifndef LORA_SYNC
#define LORA_SYNC 0x12
#endif
#ifndef LORA_INIT_RETRY_COUNT
#define LORA_INIT_RETRY_COUNT 3
#endif
#ifndef LORA_INIT_RETRY_DELAY_MS
#define LORA_INIT_RETRY_DELAY_MS 100
#endif
#ifndef METRICS_SUPPLY_VOLTAGE
#define METRICS_SUPPLY_VOLTAGE 3.30f
#endif
#ifndef LORA_PREAMBLE_SYMBOLS
#define LORA_PREAMBLE_SYMBOLS 12.0f
#endif

// Forward declaration of queue flush in UAV sketch
void flushLowQueue();

static inline float estimateLoRaToA_ms(uint8_t sf, float bwHz, uint8_t crDen, uint16_t payloadBytes) {
  return estimateLoRaToA_ms(sf, bwHz, crDen, payloadBytes, LORA_PREAMBLE_SYMBOLS);
}

static inline float estimatePacketEnergy_mJ(uint8_t tpDbm, float toaMs) {
  return estimatePacketEnergy_mJ(tpDbm, toaMs, METRICS_SUPPLY_VOLTAGE);
}

static inline void applyRadioSettings(uint8_t sf, uint8_t tp) {
  radio.standby(); delay(1);
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(sf);
  radio.setOutputPower(tp);
  radio.standby();
  Serial.print("[RADIO UAV] Applied SF=");
  Serial.print(sf);
  Serial.print(" TP=");
  Serial.println(tp);
}

static inline void hardResetLoRaUAV() {
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(25);
  digitalWrite(LORA_RST, HIGH);
  delay(120);
}

static inline bool initLoRaRadioUAV() {
  for (uint8_t i = 0; i < LORA_INIT_RETRY_COUNT; i++) {
    hardResetLoRaUAV();
    delay(LORA_INIT_RETRY_DELAY_MS);
    int16_t state = radio.begin(FREQ_MHZ, LORA_BW_KHZ, currentSF, LORA_CR_DEN, LORA_SYNC, currentTP);
    Serial.print("[BOOT UAV] LoRa init attempt ");
    Serial.print(i + 1);
    Serial.print(" state=");
    Serial.println(state);
    if (state == RADIOLIB_ERR_NONE) {
      radio.explicitHeader();
      radio.setCRC(true);
      applyRadioSettings(currentSF, currentTP);
      loraInitRetrySuccessCount++;
      return true;
    }
    delay(50);
  }
  return false;
}

static inline void softRecoverRadio(bool flushLow) {
  radio.standby(); delay(2); radio.sleep(); delay(10);
  applyRadioSettings(currentSF, currentTP);
  if (flushLow && !paramSyncActive && lowCount > 16) flushLowQueue();
  lastRadioRecoveryMs = millis();
  radioSoftRecoveryCount++;
}

#endif // UAV_RADIO_HELPERS_H
