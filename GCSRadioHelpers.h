#ifndef GCS_RADIO_HELPERS_H
#define GCS_RADIO_HELPERS_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "TelemetryProtoFix.h"

// Extern references to variables in GCS sketch
extern SX1278 radio;
extern int activeSF;
extern int activeTP;
extern HardwareSerial MetricsSerial;

#ifndef LORA_BW_KHZ
#define LORA_BW_KHZ 250.0f
#endif
#ifndef SF_MIN
#define SF_MIN 7
#endif
#ifndef SF_MAX
#define SF_MAX 12
#endif
#ifndef TP_MIN
#define TP_MIN 10
#endif
#ifndef TP_MAX
#define TP_MAX 20
#endif

static inline void applyRadioSettings(uint8_t sf, uint8_t tp) {
  if (sf < SF_MIN) sf = SF_MIN;
  if (sf > SF_MAX) sf = SF_MAX;
  if (tp < TP_MIN) tp = TP_MIN;
  if (tp > TP_MAX) tp = TP_MAX;

  activeSF = sf;
  activeTP = tp;

  radio.standby(); delay(1);
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(activeSF);
  radio.setOutputPower(activeTP);
  radio.standby();

  MetricsSerial.print("[RADIO GCS] Applied SF=");
  MetricsSerial.print(activeSF);
  MetricsSerial.print(" TP=");
  MetricsSerial.println(activeTP);
}

static inline void applyRadioSettings(uint8_t sf) {
  applyRadioSettings(sf, activeTP);
}

static inline void softRecoverRadio() {
  radio.standby(); delay(2);
  radio.sleep(); delay(10);
  applyRadioSettings(activeSF);
}

#endif // GCS_RADIO_HELPERS_H
