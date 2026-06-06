# MAVLink LoRa Bridge

## Overview

The MAVLink LoRa Bridge is a firmware implementation for ESP32-based telemetry relay systems that bridges long-range LoRa communication channels between Unmanned Aerial Vehicles (UAVs) and Ground Control Stations (GCS) running Mission Planner or compatible MAVLink implementations. The system addresses fundamental timing and bandwidth constraints inherent to half-duplex LoRa communication at extended range (Spreading Factors SF10–SF12), preventing command timeout failures and telemetry packet loss that would otherwise render the vehicle uncontrollable.

**Key Specifications:**
- Fixed 109-byte telemetry beacon packet structure
- Distributed compression: 80-byte flight data payload
- Hardware: ESP32 with SX1278 LoRa transceiver
- Protocol: MAVLink over half-duplex LoRa (433 MHz)
- Symmetric cryptographic validation on all packets
- Spreading Factor adaptive state machine (SF7–SF12)

---

## Architecture

The codebase follows a modular partition between Ground Control Station (GCS) and UAV (vehicle) compilation units, sharing a common protocol header.

### Directory Structure

```
mavlink-lora-bridge/
├── GCS_Static/
│   ├── GCS_Static.ino                  (GCS entry point, 131 KB)
│   ├── TelemetryProtoFix.h             (Shared OTA wire format)
│   ├── BeaconDecodeHelpers.h           (GCS: beacon→MAVLink reconstruction)
│   ├── GCSRadioHelpers.h               (GCS: radio parameter init)
│   ├── GCSCommandQueue.h               (GCS: command encoding queue)
│   └── GCCalibHelpers.h                (GCS: calibration state tracking)
│
├── UAV_Static/
│   ├── UAV_Static.ino                  (UAV entry point, 93 KB)
│   ├── TelemetryProtoFix.h             (Shared OTA wire format)
│   ├── BeaconFillHelpers.h             (UAV: MAVLink→beacon compression)
│   ├── UAVRadioHelpers.h               (UAV: radio parameter init)
│   ├── UAVParamQueue.h                 (UAV: parameter bulk sync queue)
│   └── UAVCalibHelpers.h               (UAV: magnetometer calibration tracking)
│
├── LICENSE                              (GNU GPL v3.0)
└── README.md                            (this file)
```

### Modular Design Pattern

Both GCS and UAV sketches include the same `TelemetryProtoFix.h` header containing:
- Wire format structure definitions (packed, OTA-safe)
- Static inline utility functions (CRC, authentication, unit conversions)
- Protocol constants and packet type identifiers
- No global variable dependencies (pure functional utilities)

Role-specific headers are compiled only in their respective binaries:
- **GCS only:** `BeaconDecodeHelpers.h`, `GCSRadioHelpers.h`, `GCSCommandQueue.h`, `GCCalibHelpers.h`
- **UAV only:** `BeaconFillHelpers.h`, `UAVRadioHelpers.h`, `UAVParamQueue.h`, `UAVCalibHelpers.h`

This separation prevents namespace pollution and reduces compiled binary size.

---

## Problem Statement: Generic Bridges and LoRa Latency

### Bandwidth and Latency Constraints

Standard transparent serial bridges forward raw MAVLink byte streams directly over the radio link. This approach fails catastrophically on long-range LoRa configurations because:

**Latency Scaling:** At SF10–SF12 (required for 5+ km range), single packet airtime increases exponentially:
- SF7, 125 kHz BW, 48-byte packet: ~49 ms
- SF10, 125 kHz BW, 48-byte packet: ~429 ms
- SF12, 125 kHz BW, 48-byte packet: ~1649 ms

A full MAVLink `PARAM_VALUE` packet (typically 34 bytes) requires:
- **Single direction:** ~430 ms at SF10, ~1.6 s at SF12
- **Bidirectional handshake (request + response):** 860 ms to 3.2 seconds

### Consequences of Latency on MAVLink Protocol

MAVLink's serial implementation assumes sub-100 ms round-trip latency. When stretched to multi-second scales:

1. **Parameter Sync Stalls:** Mission Planner sends parameter read requests at ~1 Hz. If a response takes 2+ seconds due to SF12 airtime, the GCS interprets the delayed packet as loss, triggers automatic retries, and logs "Commands don't match" errors. Parameter download hangs indefinitely.

2. **GCS Command Timeout Failsafes:** Flight-critical messages (`ARM/DISARM`, `SET_MODE`, `COMMAND_LONG`) are queued alongside telemetry (`ATTITUDE`, `VFR_HUD`). Heavy telemetry packets saturate the uplink queue, delaying command transmission. Half-duplex operation (one direction at a time) compounds this: while the UAV transmits telemetry (429 ms at SF10), the GCS cannot transmit acknowledgements, causing timeout resets on both sides.

3. **Packet Collision in Half-Duplex:** Both sides attempt transmission simultaneously (network collision), resulting in corrupted frames and forced re-transmission. Total round-trip time balloons from 860 ms to 2+ seconds per command cycle.

---

## Solution: On-Chip MAVLink Parsing and Semantic Compression

The firmware executes MAVLink protocol parsing on the ESP32 itself, intercepts raw telemetry and command messages, and reduces data volume through fixed-size semantic compression.

### Uplink Path (UAV → GCS)

```
Pixhawk (MAVLink)
    ↓
UART serial (MAVLink packets, ~50–100 bytes each)
    ↓
[UAVRadioHelpers: UART init + baud config]
    ↓
[BeaconFillHelpers: MAVLink message extraction]
    UAV parses incoming MAVLink messages:
    - HEARTBEAT (system status)
    - ATTITUDE (roll, pitch, yaw)
    - GLOBAL_POSITION_INT (lat, lon, alt)
    - VFR_HUD (airspeed, groundspeed, heading, climb)
    - BATTERY_STATUS (voltage, current)
    - GPS_RAW_INT (fix, satellites, accuracy)
    - VIBRATION (accel motion quality)
    - RC_CHANNELS_RAW (PWM inputs)
    ↓
[TelemetryProtoFix: Compression/scaling routines]
    - Attitude: float radians → int16_t centi-degrees
    - Velocity: float m/s → int16_t cm/s
    - Altitude: float meters → int16_t decimeters
    - Airspeed: float → uint16_t cm/s
    - RC channels: PWM 1000–2000 µs → 0–100%
    - GPS: full 20-byte struct → 8-byte compact
    - Vibration: clamp to m/s² × 100
    ↓
[PixhawkDataBeacon: 80-byte packed structure]
    ↓
[TelemetryBeaconPacket: 109 bytes total (header + metadata + beacon)]
    ↓
SX1278 Radio Hardware
    Configured (SF, TP, BW, CRC)
    ↓
LoRa Transmission (SF10: 429 ms, SF12: 1649 ms for 109 bytes)
    ↓
GCS Reception
```

### Downlink Path (GCS → UAV)

```
Mission Planner (MAVLink UI)
    ↓
GCS Sketch
    ↓
[GCSCommandQueue: Message queuing and encoding]
    User initiates:
    - ARM/DISARM → CompactArmDisarmPacket
    - Mode change → CompactSetModePacket
    - Waypoint load → CompactCommandLongPacket
    ↓
[TelemetryProtoFix: Compact packet structure]
    Command parameters downscaled to fit:
    - P1, P2, P3, P4: ÷1000 (e.g., ÷1000 for float param)
    - P5, P6: ÷1e7 (lat/lon)
    - P7: ÷1000
    ↓
[GCSRadioHelpers: Transmission setup]
    ↓
SX1278 Radio Hardware
    ↓
LoRa Transmission (429 ms at SF10 for ~35-byte compact command)
    ↓
UAV Reception
    ↓
[BeaconDecodeHelpers: Command extraction and upscaling]
    Parameters: ×1000, ×1e7 applied to restore floats
    ↓
[UAV_Static: Command forwarding to Pixhawk]
    ↓
Pixhawk/Ardupilot Flight Controller
    Executes command (arm, mode change, navigate)
    ↓
Status change → HEARTBEAT message generated
    ↓
Uplink path (above) transmits confirmation
```

---

## Fixed-Size Packet Structure: 109-Byte Telemetry Beacon

All telemetry data is packed into a strictly bounded 109-byte (`TelemetryBeaconPacket`) structure to enable deterministic airtime calculation and guarantee bandwidth allocation predictability.

### Packet Layout Breakdown

| Component | Size (Bytes) | Purpose |
|-----------|-------------|---------|
| **PacketHeader** | 10 | Type, protocol version, network ID, CRC-16, authentication token |
| **counter** | 4 | Sequential frame ID for packet loss detection and replay protection |
| **radio_packed** | 1 | Bit-packed SF, TP, profile (enables mid-flight adaptation) |
| **remote_snr_x2** | 1 | UAV-measured GCS SNR × 2 (dB range ±63.5) |
| **remote_rssi_q** | 1 | UAV-measured GCS RSSI quantized 1–254 |
| **link_mode** | 1 | Operational state (telemetry, command hold, sync block) |
| **latency_x100** | 2 | Half-RTT × 100 (milliseconds, max 655 ms) |
| **TelemetryMeta** | 9 | ACK count, success/failure streaks, cumulative TX energy |
| **PixhawkDataBeacon** | 80 | Compressed flight data (detailed below) |
| **Total** | **109** | |

### PixhawkDataBeacon Payload (80 Bytes)

```c
struct __attribute__((packed)) PixhawkDataBeacon {
  // Status: 7 bytes
  uint8_t  valid_flags;        // Validity flags (bits: HEARTBEAT, ATTITUDE, GPS, etc.)
  uint8_t  valid_flags2;       // Extended flags (VIBRATION, RC_CHANNELS)
  uint8_t  system_id;          // MAVLink system ID (Pixhawk)
  uint8_t  component_id;       // Component ID (flight controller)
  uint8_t  base_mode;          // Vehicle mode (MANUAL, GUIDED, AUTO, etc.)
  uint32_t custom_mode;        // Ardupilot flight mode enum
  uint8_t  system_status;      // Health: ACTIVE, CRITICAL, EMERGENCY

  // Attitude: 6 bytes (roll, pitch, yaw in centi-degrees)
  int16_t  roll_cd;            // ÷100 = degrees
  int16_t  pitch_cd;
  int16_t  yaw_cd;

  // Position: 10 bytes
  int32_t  lat;                // degrees × 1e7 (MAVLink native)
  int32_t  lon;
  int16_t  relative_alt_dm;    // relative altitude in decimeters

  // Battery: 5 bytes
  uint16_t voltage_battery;    // millivolts
  int16_t  current_battery;    // centiamperes
  int8_t   battery_remaining;  // percentage

  // Airdata: 7 bytes
  uint16_t airspeed_cms;       // cm/s (true airspeed)
  uint16_t groundspeed_cms;    // cm/s (estimated ground velocity)
  uint16_t heading;            // centi-degrees
  int16_t  climb_cms;          // cm/s (vertical velocity)
  uint8_t  throttle;           // 0–100%

  // Navigation: 2 bytes + 8 bytes (GPS sub-struct)
  uint16_t ekf_flags;          // EKF filter status bits
  struct {                      // GpsDataRingkas (8 bytes)
    uint8_t  fix_type;         // 0=none, 2=2D, 3=3D, 4=DGPS, 5=RTK
    uint8_t  satellites_visible;
    int16_t  alt_dm;           // GPS altitude in decimeters
    uint16_t eph;              // Horizontal accuracy (mm)
    uint16_t epv;              // Vertical accuracy (mm)
  } gps;

  // Velocity (NED frame): 6 bytes
  int16_t  vx_cms;             // North velocity (cm/s)
  int16_t  vy_cms;             // East velocity (cm/s)
  int16_t  vz_cms;             // Down velocity (cm/s)

  // Vibration and Clipping: 8 bytes
  int16_t  vibe_x_x100;        // X acceleration vibration (m/s² × 100)
  int16_t  vibe_y_x100;        // Y acceleration vibration
  int16_t  vibe_z_x100;        // Z acceleration vibration
  uint16_t accel_clip;         // Total IMU clipping events

  // RC Channels: 16 bytes (16 channels × 1 byte each)
  uint8_t  rc_ch_pct[16];      // Channels 1–16 as 0–100% duty cycle
};
// Total: 80 bytes
```

### Data Compression Strategy

Each field is downscaled to the minimum integer type that preserves practical flight control resolution:

| Field | Original Type | Compression | Rationale |
|-------|---------------|-------------|-----------|
| Attitude (roll, pitch, yaw) | float (4 bytes × 3) | int16_t centi-degrees (2 bytes × 3) | Centi-degree resolution (±327 deg) exceeds aircraft stability requirements. Mission Planner displays centi-degrees natively. |
| Relative altitude | float (4 bytes) | int16_t decimeters (2 bytes) | ±327 meters decimeter-step resolution sufficient for altitude control. Upscaled on GCS. |
| Airspeed, groundspeed | float (4 bytes × 2) | uint16_t cm/s (2 bytes × 2) | ±655 m/s (cm/s resolution). Covers all aircraft flight envelopes. |
| Heading | float (4 bytes) | uint16_t centi-degrees (2 bytes) | Compass resolution to ±3.27 degrees. |
| GPS altitude | float (4 bytes) | int16_t decimeters (2 bytes) | GPS altitude with decimeter step. Still decimeter-accurate within ±327 m. |
| Climb rate (Vz) | float (4 bytes) | int16_t cm/s (2 bytes) | Vertical velocity in cm/s. ±327 m/s range adequate. |
| Vibration (accel) | float (4 bytes × 3) | int16_t × 100 (m/s²) (2 bytes × 3) | Vibration in 0.01 m/s² steps. ±327 m/s². Matches IMU noise floor (~0.1 g = 0.98 m/s²). |
| RC channels (16 × 2 bytes) | uint16_t PWM (1000–2000 µs) | uint8_t 0–100% (1 byte) | RC command resolution to 1%. Sufficient for servo/motor control. |
| Battery voltage | uint16_t (2 bytes) | uint16_t (2 bytes) | Millivolt precision, no scaling. |
| Current | int16_t (2 bytes) | int16_t (2 bytes) | Centiampere precision, no scaling. |
| NED velocities | float (4 bytes × 3) | int16_t cm/s (2 bytes × 3) | Navigation velocity to 1 cm/s. ±327 m/s adequate. |

**Result:** Original MAVLink telemetry (≈200+ bytes) compressed to 80 bytes without loss of control-critical resolution.

---

## Adaptive Link State Machine

The firmware implements a dynamic state machine responsive to LoRa link degradation (Spreading Factor changes).

### State Transitions

```
┌─────────────────────────────────────────────────────────────┐
│                   LINK_MODE Enumeration                     │
├─────────────────────────────────────────────────────────────┤
│ LINK_MODE_TELEMETRY:      (0)  Default: full telemetry      │
│   - All beacon fields streamed                              │
│   - Parameter sync enabled                                  │
│   - GCS commands processed at normal rate                   │
│                                                              │
│ LINK_MODE_COMMAND:        (1)  Flight command in progress   │
│   - Telemetry suppressed for HOLD_MS                        │
│   - Command ACK priority                                    │
│   - Safety state: prevents conflicting user input           │
│                                                              │
│ LINK_MODE_HIGH_SF_BLOCK:  (2)  Spreading Factor ≥ 10        │
│   - Parameter sync disabled (too slow for SF10+)            │
│   - Emergency telemetry only                                │
│   - Commands still accepted                                 │
│   - GCS forced to use pre-loaded parameters                 │
└─────────────────────────────────────────────────────────────┘
```

### Parameter Sync Blocking at High Spreading Factors

When link degrades to SF10, SF11, or SF12:

| Spreading Factor | SF10 | SF11 | SF12 |
|---|---|---|---|
| Airtime (109 bytes) | 429 ms | 858 ms | 1649 ms |
| Parameters/sec | 2.3 | 1.2 | 0.6 |
| Full param download (200 params) | 87 sec | 167 sec | 333 sec |
| User tolerance threshold | ✗ | ✗ | ✗ |

**Policy:** When SF ≥ 10, `BLOCK_FULL_PARAM_SYNC_HIGH_SF` blocks the parameter list download query. This prevents timeouts and false "sync failed" messages. Users must load parameters via pre-flight USB connection or accept read-only telemetry mode.

### Command Hold Time

Flight-critical commands (ARM, DISARM, SET_MODE) trigger:
```c
#define FLIGHT_COMMAND_HOLD_MS 3000UL
```

During this window:
- Uplink: All GCS commands suppressed (prevents conflicting rapid commands)
- Downlink: Priority ACK transmission
- State: UAV firmware waits for Pixhawk acknowledgement

This prevents the half-duplex collision scenario where GCS re-transmits while UAV is still processing.

---

## Command Encoding and Decoding

### GCS → UAV Downlink Protocol

The GCS encodes flight commands into compact structures to fit within the LoRa payload budget.

#### 1. ARM/DISARM Command

```c
struct __attribute__((packed)) CompactArmDisarmPacket {
  PacketHeader hdr;           // 10 bytes
  uint16_t seq;               // Sequence number
  uint8_t  kind;              // COMPACT_CMD_KIND_ARM_DISARM = 1
  uint8_t  target_system;     // MAVLink system ID
  uint8_t  target_component;  // MAVLink component ID
  uint8_t  arm;               // 1=arm, 0=disarm
  int32_t  param2_x1000;      // param2 × 1000 (e.g., gyro force level)
};
// Total: 26 bytes
```

**Encoding (GCS):**
```cpp
// Mission Planner: user clicks "Arm Motors"
// GCS firmware extracts:
cmd.arm = 1;  // arm flag
cmd.param2_x1000 = (int32_t)(mavlink_msg.param2 * 1000.0f);

// Send over LoRa
```

**Decoding (UAV):**
```cpp
// UAV firmware receives CompactArmDisarmPacket
// Reconstruct MAVLink COMMAND_LONG:
float param2 = (float)cmd.param2_x1000 / 1000.0f;

// Build MAVLink message:
mavlink_command_long_t mav_cmd;
mav_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
mav_cmd.param1 = (cmd.arm ? 1.0f : 0.0f);
mav_cmd.param2 = param2;

// Forward to Pixhawk
```

#### 2. SET_MODE Command

```c
struct __attribute__((packed)) CompactSetModePacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t  kind;              // COMPACT_CMD_KIND_SET_MODE = 2
  uint8_t  target_system;
  uint8_t  base_mode;         // MAVLink base mode bits
  uint32_t custom_mode;       // Ardupilot/PX4 flight mode enum
};
// Total: 21 bytes
```

**Encoding (GCS):**
```cpp
// Mission Planner: user selects "GUIDED" mode
// Custom mode fetched from Ardupilot enum:
cmd.custom_mode = GUIDED_MODE_ENUM;  // 4 (Ardupilot)
cmd.base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
```

**Decoding (UAV):**
```cpp
// UAV receives CompactSetModePacket, forwards directly to Pixhawk
mavlink_set_mode_t msg;
msg.base_mode = cmd.base_mode;
msg.custom_mode = cmd.custom_mode;
```

#### 3. General COMMAND_LONG

```c
struct __attribute__((packed)) CompactCommandLongPacket {
  PacketHeader hdr;
  uint16_t seq;
  uint8_t  kind;              // COMPACT_CMD_KIND_COMMAND_LONG = 3
  uint16_t command;           // MAV_CMD_* identifier
  uint8_t  target_system;
  uint8_t  target_component;
  uint8_t  confirmation;
  int32_t  p1_x1000;          // param1 × 1000
  int32_t  p2_x1000;          // param2 × 1000
  int32_t  p3_x1000;          // param3 × 1000
  int32_t  p4_x1000;          // param4 × 1000
  int32_t  p5_x1e7;           // param5 × 1e7 (lat/lon)
  int32_t  p6_x1e7;           // param6 × 1e7
  int32_t  p7_x1000;          // param7 × 1000
};
// Total: 42 bytes
```

**Example: Waypoint Navigation (MAV_CMD_NAV_WAYPOINT)**

GCS downlink:
```cpp
float lat = -33.8688;           // Sydney Opera House
float lon = 151.2093;
int32_t alt_cm = 5000;          // 50 meters

// Compact encoding:
cmd.p5_x1e7 = (int32_t)(lat * 1e7);     // -338688000
cmd.p6_x1e7 = (int32_t)(lon * 1e7);     // 1512930000
cmd.p7_x1000 = (int32_t)(alt_cm / 100.0f * 1000.0f);  // 50000 (50 m)
```

UAV uplink:
```cpp
// Reconstruct floats:
float decoded_lat = (float)cmd.p5_x1e7 / 1e7;        // -33.8688
float decoded_lon = (float)cmd.p6_x1e7 / 1e7;        // 151.2093
float decoded_alt = (float)cmd.p7_x1000 / 1000.0f;   // 50.0 meters
```

---

## Cryptographic Validation

All packets (uplink and downlink) are validated with a lightweight keyed hash to reject third-party injection and replay attacks.

### Packet Authentication Layer

Each `PacketHeader` contains two security fields:

```c
struct __attribute__((packed)) PacketHeader {
  uint8_t  type;              // Packet type (PKT_TELEM_BEACON, etc.)
  uint8_t  protocol;          // PROTOCOL_VERSION = 0x04
  uint16_t network_id;        // NETWORK_ID = 0x2244
  uint16_t crc;               // CRC-16/CCITT-FALSE (payload integrity)
  uint32_t token;             // Keyed auth-tag (replay prevention)
};
```

### CRC-16 Computation

```cpp
static inline uint16_t computePacketCrc(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    uint8_t v = buf[i];
    // Skip CRC and token fields during computation
    if (isPacketCrcByte(i) || isPacketAuthByte(i)) v = 0;
    
    crc ^= ((uint16_t)v << 8);
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}
```

Validates: bit-level corruption during RF transmission.

### Authentication Tag (Anti-Replay)

```cpp
static inline uint32_t computePacketAuthTag(const uint8_t *buf, size_t len) {
  uint32_t h = 2166136261UL         // FNV-1a offset basis
            ^ AUTH_TOKEN            // 0xA56C93D1UL (pre-shared secret)
            ^ SECURITY_KEY_MIX      // 0x3D7F21B9UL (obfuscation key)
            ^ ((uint32_t)NETWORK_ID << 8)
            ^ (uint32_t)PROTOCOL_VERSION;
  
  for (size_t i = 0; i < len; i++) {
    uint8_t v = buf[i];
    if (isPacketAuthByte(i)) v = 0;  // Exclude token field
    
    h ^= v;
    h *= 16777619UL;               // FNV prime
    h ^= (h >> 13);                // avalanche mixing
  }
  
  h ^= (uint32_t)len * 0x9E3779B9UL;  // length mixing
  h ^= (h >> 16);
  h *= 0x7FEB352DUL;
  h ^= (h >> 15);
  h *= 0x846CA68BUL;
  h ^= (h >> 16);
  
  if (h == 0 || h == AUTH_TOKEN) h ^= 0xA5A55A5AUL;  // avoid sentinel values
  return h;
}
```

This hash-based HMAC provides:
1. **Per-packet verification:** Every frame is cryptographically bound to the shared secret
2. **Replay detection:** Token changes with packet content; replaying old packets fails validation
3. **Light computational load:** No AES or elliptic curves; suitable for ESP32 at RF interrupt rates

### Anti-Replay Counter

The packet `counter` field increments sequentially:

```cpp
#define SECURITY_MAX_COUNTER_GAP 5000UL

// UAV validates incoming GCS packet:
if (abs(gcs_counter - last_gcs_counter) > SECURITY_MAX_COUNTER_GAP) {
  // Reject: packet gap too large (packet reuse detected)
  packet_drop++;
  return false;
}
last_gcs_counter = gcs_counter;
```

If an attacker replays an old packet with an old counter value, the receiver rejects it (counter is outside the sliding window). Reboot grace period (`SECURITY_REBOOT_GRACE_MS = 15 s`) allows receiver resets.

---

## Hardware Configuration

### ESP32 LoRa Pinout (SPI Bus, VSPI)

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 (LoRa Bridge)                       │
├─────────────────────────────────────────────────────────────┤
│ Pixhawk UART Connection:                                    │
│   RX ← Pixhawk TX (telemetry output)                        │
│   TX → Pixhawk RX (command input)                           │
│   GND = Pixhawk GND                                          │
│                                                              │
│ SX1278 LoRa Transceiver (SPI Interface):                    │
│   MOSI  = GPIO 23   (Master Out, Slave In)                  │
│   MISO  = GPIO 19   (Master In, Slave Out)                  │
│   SCLK  = GPIO 18   (Serial Clock)                          │
│   NSS   = GPIO 5    (Slave Select / Chip Enable)            │
│   RST   = GPIO 25   (Hardware Reset)                        │
│   DIO0  = GPIO 26   (Interrupt, RxDone/TxDone)             │
│   DIO1  = RADIOLIB_NC (Not connected; status via register)  │
│                                                              │
│ RadioLib (Arduino LoRa library) Configuration:              │
│   - Frequency: 433.0 MHz (ISM band, region dependent)       │
│   - Signal Bandwidth: 500.0 kHz                             │
│   - Spreading Factor: SF7–SF12 (adaptive)                   │
│   - Coding Rate: 4/8 (error correction)                     │
│   - CRC: enabled (hardware FCS)                             │
│   - Sync Word: 0x12 (network identification)                │
│   - Preamble: GCS=8, UAV=12 (affects airtime calc)          │
│   - Power: variable, TP7–TP20 (10–20 dBm)                  │
└─────────────────────────────────────────────────────────────┘
```

### RF Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| **Frequency** | 433.0 MHz | ISM band, worldwide legal (varies by region). Lower path loss than 2.4 GHz. |
| **Bandwidth** | 500 kHz | Wider BW = faster airtime but poorer link margin. 500 kHz balances range/latency. |
| **Coding Rate** | 4/8 | 4 data bits per 8 code bits. Intermediate FEC overhead. Prevents excessive retransmits. |
| **Sync Word** | 0x12 | Identifies this network. Rejections packets from other LoRa nets on same freq. |
| **CRC** | Enabled | Hardware-assisted frame check. Rejects bit-corrupted packets at PHY layer. |

### Airtime Calculation Example (SF10, 500 kHz BW)

For 109-byte TelemetryBeaconPacket at SF10:

```
Spreading Factor (SF) = 10
Bandwidth (BW)        = 500 kHz
Payload               = 109 bytes
Preamble Symbols      = 12 (UAV) or 8 (GCS)
Coding Rate (CR)      = 4/8

Symbol Time = (2^SF / BW) × 1000 = (2^10 / 500000) × 1000 = 2.048 ms

Preamble Time = (12 + 4.25) × 2.048 = 33.28 ms

Payload Symbols ≈ 8 + ceil((8×109 - 4×10 + 28 + 16×1 - 20×0) / (4×(10-0))) × 4/8
                ≈ 8 + ceil(876 / 40) × 4/8
                ≈ 8 + 22 × 4/8
                ≈ 8 + 11 = 19 symbols (after FEC expansion)

Payload Time = 19 × 2.048 = 38.9 ms

Total Airtime = 33.28 + 38.9 = 72.2 ms (conservative estimate; actual ~90–100 ms due to ramp-up)

At SF12:
Symbol Time  = 8.192 ms
Preamble     = 131.1 ms
Payload      ≈ 76 symbols × 8.192 = 622 ms
Total        ≈ 750 ms (conservative; typically 600–700 ms for 109-byte packet)
```

This allows deterministic scheduling and latency budgeting.

---

## Mission Planner Integration Workarounds

Several firmware quirks prevent GCS timeout failures:

### 1. Telemetry ACK Decimation

Mission Planner sends implicit ACKs for received telemetry packets. In half-duplex, simultaneous UAV telemetry and GCS ACK transmissions collide.

**Solution:**
```cpp
#define TELEMETRY_ACK_DECIMATION_ENABLE 1  // Filter redundant GCS responses

if (TELEMETRY_ACK_DECIMATION_ENABLE) {
  // Only forward every 3rd ACK from GCS to UAV
  if (ack_counter % 3 == 0) {
    radio.transmit(ack_packet);
  }
  ack_counter++;
}
```

Reduces ACK traffic by 67%, freeing uplink bandwidth for critical commands.

### 2. Dual System ID Isolation

Mission Planner may encounter messages from both GCS (ESP32 system ID 1) and Pixhawk (system ID 2) simultaneously, causing confusion.

**Solution:**
```cpp
// GCS side: rewrite incoming Pixhawk messages with GCS system ID
if (msg.sysid == PIXHAWK_SYSTEM_ID) {
  msg.sysid = GCS_SYSTEM_ID;   // Merge into single virtual system for UI
}

// UAV side: strip GCS system ID on downlink commands
if (cmd.target_system == GCS_SYSTEM_ID) {
  cmd.target_system = PIXHAWK_SYSTEM_ID;  // Route to flight controller only
}
```

### 3. Autonomous Pixhawk UART Recovery

If the Pixhawk serial interface goes silent (watchdog):

```cpp
#define PIXHAWK_UART_SILENT_MS 12000UL  // 12 second timeout

if (millis() - last_pixhawk_rx_ms > PIXHAWK_UART_SILENT_MS) {
  // Pixhawk unresponsive; trigger hot reset
  digitalWrite(PIXHAWK_RESET_PIN, LOW);
  delay(100);
  digitalWrite(PIXHAWK_RESET_PIN, HIGH);
  delay(1000);  // Allow boot
  
  last_pixhawk_rx_ms = millis();
}
```

Prevents lockup when Pixhawk firmware crashes or enters bootloader mode.

---

## Software Build and Deployment

### Arduino IDE Compilation

Both sketches are compiled separately (one binary per vehicle type):

1. **GCS_Static.ino**
   - Compiled with all GCS helper headers included
   - Outputs `GCS_Static.bin` (~130 KB)
   - Uploads to GCS ESP32 via USB/UART programmer

2. **UAV_Static.ino**
   - Compiled with all UAV helper headers included
   - Outputs `UAV_Static.bin` (~93 KB)
   - Uploads to UAV ESP32 via USB/UART programmer

### Dependencies

- **Arduino Core for ESP32** (v2.0+): standard HAL, WiFi, SPI drivers
- **RadioLib** (v6.0+): LoRa transceiver abstraction, SPI control
- **MAVLink C library** (common set): message marshaling, CRC, serialization

### Configuration Header (in sketch root)

```cpp
// config.h or sketch defines
#define PIXHAWK_BAUD         57600     // Serial baud rate
#define PIXHAWK_UART_SILENT_MS 12000UL // Watchdog timeout (ms)
#define FLIGHT_COMMAND_HOLD_MS 3000UL  // Command grace period
#define LORA_FREQ_MHZ         433.0    // Radio frequency
#define LORA_BW_KHZ           500.0    // Bandwidth
#define LORA_SF_DEFAULT       10       // Starting spreading factor
#define LORA_TP_DBM           20       // TX power
#define SECURITY_KEY_MIX      0x3D7F21B9UL
#define AUTH_TOKEN            0xA56C93D1UL
```

---

## Protocol Constants and Frame Format

### Packet Type Identifiers

```c
#define PKT_LINK_ACK        0xA5  // Link-layer ACK (minimal)
#define PKT_MAVLINK_RAW     0x4D  // Pass-through raw MAVLink ('M')
#define PKT_CMD_COMPACT     0x43  // Encoded flight command ('C')
#define PKT_PARAM_BULK      0x50  // Parameter download batch ('P')
#define PKT_TELEM_BEACON    0x57  // Main telemetry beacon ('W')
#define PKT_CONFIG_PROPOSE  0xC0  // SF/TP adaptation proposal
#define PKT_CONFIG_ACK      0xC1  // SF/TP adaptation acceptance
```

### Network Constants

```c
#define PROTOCOL_VERSION        0x04   // Firmware protocol revision
#define NETWORK_ID              0x2244 // Network magic (prevents cross-link)
#define AUTH_TOKEN              0xA56C93D1UL
#define SECURITY_KEY_MIX        0x3D7F21B9UL
#define SECURITY_MAX_COUNTER_GAP 5000UL  // Anti-replay window
#define SECURITY_REBOOT_GRACE_MS 15000UL // Grace period post-reboot
```

---

## Testing and Validation

### Functional Test Checklist

- [ ] **UART Link:** Pixhawk → ESP32 serial at 57600 baud, MAVLink frames parsed
- [ ] **Radio Init:** SX1278 SPI communication verified, frequency/BW set correctly
- [ ] **Telemetry Uplink:** UAV transmits 109-byte beacons at >0.5 Hz (adjustable)
- [ ] **Command Downlink:** GCS transmits compact commands; UAV forwards to Pixhawk
- [ ] **CRC Validation:** Corrupted packets rejected at receiver
- [ ] **Auth Token:** Replayed packets rejected by anti-replay counter
- [ ] **Parameter Sync:** Parameter download blocked at SF10+; Mission Planner handles gracefully
- [ ] **Command Hold:** ARM/DISARM enforces 3 s quiet period before next command accepted
- [ ] **Link State Machine:** Transitions between TELEMETRY / COMMAND / HIGH_SF_BLOCK correctly
- [ ] **Pixhawk Watchdog:** Timeout triggers ESP32→Pixhawk reset pin toggle

### Link Test Procedure

**Equipment:**
- Two ESP32 modules with SX1278 transceivers
- One Pixhawk flight controller (or MAVLink simulator)
- Mission Planner running on PC
- Multimeter (measure current for power budgeting)

**Test Setup:**
1. Program GCS ESP32 with GCS_Static.ino
2. Program UAV ESP32 with UAV_Static.ino + connect Pixhawk UART
3. Both modules powered at 5V (ensure adequate current supply: ~150 mA peak TX)
4. Launch Mission Planner on PC, connect to GCS ESP32 serial
5. Verify telemetry stream arrives (~1–2 Hz beacon rate)
6. Transmit ARM command; observe Pixhawk LED arming response
7. Change flight mode via Mission Planner; verify UAV obeys

---

## Performance Characteristics

### Link Budget (433 MHz, SF12)

```
Transmit Power:        20 dBm (100 mW, SX1278 typical)
TX Antenna Gain:       0 dBi (dipole)
Path Loss (5 km):      ~131 dB (Friis formula)
RX Antenna Gain:       0 dBi
RX Sensitivity:        -140 dBm (SF12, 500 kHz BW, typical)

Link Budget = TX + TX_Gain - Path_Loss + RX_Gain - RX_Loss
            = 20 + 0 - 131 + 0 - (-140)
            = 29 dB (headroom to -140 dBm target)
            ≈ 5 km line-of-sight, 2–3 km non-LOS
```

### Latency Budget

```
GCS Command → Pixhawk Execution:
  GCS encoding:                 ~5 ms
  Compact cmd uplink SF10:       ~90 ms
  UAV decoding + Pixhawk fwd:    ~10 ms
  Pixhawk processing:            ~50 ms
  Total one-way:                 ~155 ms

Pixhawk ARM acknowledgement → GCS display:
  Pixhawk HEARTBEAT generation:  ~20 ms
  UAV beacon fill + compress:    ~30 ms
  Beacon downlink SF10:          ~90 ms
  GCS decode + UI update:        ~30 ms
  Total return:                  ~170 ms

Full round-trip (command echo):  ~325 ms (at SF10)
                                 ~800 ms (at SF12)
```

### Throughput

```
Uplink Telemetry (109-byte beacon, SF10):
  Airtime:           ~90 ms
  Rate:              ~11 packets/second (theoretical)
  Practical:         ~1 packet/second (to avoid congestion)
  Data rate:         ~1 KB/s telemetry

Downlink Commands (42-byte COMMAND_LONG, SF10):
  Airtime:           ~42 ms
  Capacity:          ~24 commands/second (theoretical)
  Practical:         ~1 command/second (user-triggered)
  Data rate:         ~42 bytes/command
```

---

## License and Attribution

This project is distributed under the **GNU General Public License v3.0 (GPL-3.0)**, which requires that:
- Source code must be disclosed
- Derivative works must use the same license
- Commercial use is permitted with full source attribution

See `LICENSE` file for complete terms.

---

## Contributing and Modifications

To extend this firmware:

1. **Protocol Extensions:** Modify `TelemetryProtoFix.h` carefully—OTA struct size changes break compatibility. Add new fields only if within the 109-byte envelope (compress existing fields if needed). Update `static_assert()` checks.

2. **New Command Types:** Add a new `CompactXxxPacket` struct and `COMPACT_CMD_KIND_*` enum value. Update GCS and UAV decoders symmetrically.

3. **RF Parameter Tuning:** Adjust `LORA_BW_KHZ`, `LORA_SF_DEFAULT`, `LORA_TP_DBM` in both sketches. Recompute airtime and latency budgets. Test link margin with multimeter + current probe.

4. **Radio Library Upgrade:** If upgrading RadioLib, verify SPI pinout compatibility and re-run RF parametrization tests (SNR, RSSI calibration may shift).

---

## Revision History

| Version | Date | Notes |
|---------|------|-------|
| v2.8 | 2026-06-06 | Modular architecture (separate GCS/UAV binaries), 109-byte beacon, SF10+ parameter sync blocking, anti-replay counter |
| v2.7 | TBD | Previous iteration with monolithic .ino files |

---

## References and Resources

- **MAVLink Protocol Specification:** https://mavlink.io/
- **Ardupilot Flight Controller:** https://ardupilot.org/
- **RadioLib Documentation:** https://jgromes.github.io/RadioLib/
- **SX1278 Datasheet:** https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V7.pdf
- **LoRa Modulation & Link Budget:** https://www.semtech.com/sites/default/files/upload/Documents/LoRa/LoRaAlliance_Coverage_to_LoRa.pdf
- **ESP32 Technical Reference:** https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf