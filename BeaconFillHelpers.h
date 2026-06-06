#ifndef BEACON_FILL_HELPERS_H
#define BEACON_FILL_HELPERS_H

#include <stdint.h>
#include <math.h>
#include "TelemetryProtoFix.h"

// Forward declarations of UAV global states
struct PixhawkDataFull;
extern PixhawkDataFull pixDataFull;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

inline int16_t clampInt16FromLong(long v) {
  if (v > 32767L) return 32767;
  if (v < -32768L) return -32768;
  return (int16_t)v;
}

inline uint16_t clampUint16FromFloat(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 65535.0f) return 65535;
  return (uint16_t)(v + 0.5f);
}

inline int16_t clampInt16FromFloat(float v) {
  if (v >= 32767.0f) return 32767;
  if (v <= -32768.0f) return -32768;
  return (int16_t)(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
}

inline int16_t radToCentiDegInt16(float rad) {
  float cd = rad * 18000.0f / PI;
  return clampInt16FromFloat(cd);
}

inline int16_t mmToDecimeterInt16(int32_t mm) {
  long dm = (mm >= 0) ? ((long)mm + 50L) / 100L : ((long)mm - 50L) / 100L;
  return clampInt16FromLong(dm);
}

inline uint8_t clampPctToU8(uint16_t v) {
  if (v > 100) return 100;
  return (uint8_t)v;
}

inline void fillBeacon(PixhawkDataBeacon &d) {
  memset(&d, 0, sizeof(d));

  // ── Validity flags ──────────────────────────────
  d.valid_flags  = (uint8_t)(pixDataFull.valid_flags & 0xFF);
  d.valid_flags2 = (uint8_t)(pixDataFull.valid_flags2 & 0xFF);

  // ── Core status ─────────────────────────────────
  d.system_id     = pixDataFull.system_id;
  d.component_id  = pixDataFull.component_id;
  d.base_mode     = pixDataFull.base_mode;
  d.custom_mode   = pixDataFull.custom_mode;
  d.system_status = pixDataFull.system_status;

  // ── Attitude ────────────────────────────────────
  d.roll_cd  = radToCentiDegInt16(pixDataFull.roll);
  d.pitch_cd = radToCentiDegInt16(pixDataFull.pitch);
  d.yaw_cd   = radToCentiDegInt16(pixDataFull.yaw);

  // ── Position ────────────────────────────────────
  d.lat             = pixDataFull.lat;
  d.lon             = pixDataFull.lon;
  d.relative_alt_dm = mmToDecimeterInt16(pixDataFull.relative_alt_mm);

  // ── Battery ─────────────────────────────────────
  d.voltage_battery   = pixDataFull.voltage_battery;
  d.current_battery   = pixDataFull.current_battery;
  d.battery_remaining = pixDataFull.battery_remaining;

  // ── Airdata ─────────────────────────────────────
  d.airspeed_cms    = clampUint16FromFloat(pixDataFull.airspeed * 100.0f);
  d.groundspeed_cms = clampUint16FromFloat(pixDataFull.groundspeed * 100.0f);
  d.heading         = clampUint16FromFloat((float)pixDataFull.heading * 100.0f);
  d.climb_cms       = clampInt16FromFloat(pixDataFull.climb * 100.0f);
  d.throttle        = clampPctToU8(pixDataFull.throttle);

  // ── EKF + GPS ───────────────────────────────────
  d.ekf_flags               = pixDataFull.ekf_flags;
  d.gps.fix_type            = pixDataFull.gps.fix_type;
  d.gps.satellites_visible  = pixDataFull.gps.satellites_visible;
  d.gps.alt_dm              = mmToDecimeterInt16(pixDataFull.gps.alt_mm);
  d.gps.eph                 = pixDataFull.gps.eph;
  d.gps.epv                 = pixDataFull.gps.epv;

  // ── NED Velocity [NEW] ───────────────────────────
  if (pixDataFull.valid_flags & VALID_LOCAL_VEL) {
    d.vx_cms = pixDataFull.vx;   // GLOBAL_POSITION_INT vx is already in cm/s
    d.vy_cms = pixDataFull.vy;
    d.vz_cms = pixDataFull.vz;
  }

  // ── Vibration [NEW] ─────────────────────────────
  if (pixDataFull.valid_flags2 & VALID2_VIBRATION) {
    // Clamp float vibe to int16_t x100 range (±327.67 m/s²)
    auto clampVibe = [](float v) -> int16_t {
      float s = v * 100.0f;
      if (s >  32767.0f) return  32767;
      if (s < -32768.0f) return -32768;
      return (int16_t)(s >= 0.0f ? (s + 0.5f) : (s - 0.5f));
    };
    d.vibe_x_x100 = clampVibe(pixDataFull.vibe_x);
    d.vibe_y_x100 = clampVibe(pixDataFull.vibe_y);
    d.vibe_z_x100 = clampVibe(pixDataFull.vibe_z);
    // Sum all three accel clip counters into a single uint16_t
    uint32_t totalClip = pixDataFull.accel_clip_0 + pixDataFull.accel_clip_1 + pixDataFull.accel_clip_2;
    d.accel_clip = (totalClip > 65535UL) ? 65535U : (uint16_t)totalClip;
  }

  // ── RC Channels 1-16 [NEW] ──────────────────────
  if (pixDataFull.valid_flags2 & VALID2_RC_CHANNELS) {
    for (uint8_t i = 0; i < 16; i++) {
      d.rc_ch_pct[i] = rcRawToPct(pixDataFull.rc_channels[i]);
    }
  }
}

#endif // BEACON_FILL_HELPERS_H
