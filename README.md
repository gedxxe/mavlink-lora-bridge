
# LoRa-Telemetry-Pixhawk-Ardupilot

An optimized, MAVLink-aware half-duplex LoRa bridge (ESP32 + SX1278) engineered to prevent Mission Planner timeout crashes and telemetry stalls. Features 78-byte fixed semantic compression and dynamic command prioritization up to SF12.

---

## The Problem with Generic Transparent Bridges

Standard transparent serial bridges completely choke when passing raw MAVLink streams over long-range LoRa links. When operating at high Spreading Factors (`SF10` to `SF12`), the effective throughput drops drastically. 

This latency causes severe bottlenecks in half-duplex setups, leading to:
* **Parameter Sync Stalls:** Mission Planner misinterprets delayed packets as packet loss, triggers endless retries, and causes "Commands don't match" errors or complete parameter sync failure.
* **GCS Timeout Failsafes:** Heavy telemetry packets (`ATTITUDE`, `VFR_HUD`) saturate the downlink pipeline. Critical Ground Control Station (GCS) commands or acknowledgement (ACK) packets get trapped in the buffer queue, triggering autopilot failsafes.

This project solves these issues by executing **on-chip MAVLink parsing** on the ESP32 (utilizing `RadioLib` and custom abstractions in `TelemetryProtoFix.h`). The firmware intercepts, extracts, and serializes telemetry data into downscaled packed structures before transmission over the air.

---

## 78-Byte Fixed Semantic Compression

Instead of streaming raw MAVLink binaries, the UAV node extracts essential flight parameters and packs them into a strictly bounded **78-byte** structure (`TelemetryBeaconPacket`). The GCS node receives this packet, decompresses it, and reconstructs valid, standardized MAVLink streams locally before piping them into Mission Planner.

### `TelemetryBeaconPacket` Structure Breakdown (78 Bytes Fixed)

| Struct Component | Size | Engineering & Downscaling Implementation |
| :--- | :--- | :--- |
| `PacketHeader hdr` | 10 Bytes | Contains `type`, `protocol`, `network_id` (`0x2244`), `crc`, and validation tokens. |
| `uint32_t counter` | 4 Bytes | Sequential frame counter used for precise hardware link packet loss calculations. |
| Packed RF Metrics | 3 Bytes | Includes `radio_packed` (bit-packed SF, TP, and profile in 1 byte), `remote_snr_x2`, and `remote_rssi_q` for real-time downlink health tracking. |
| Link Mode & Latency | 3 Bytes | `link_mode` (1 byte) for state tracking, and `latency_x100` (2 bytes) for round-trip time (RTT) telemetry. |
| `TelemetryMeta meta` | 9 Bytes | Diagnostic research fields: `cnt_ack`, `success_streak`, `fail_streak`, and cumulative Tx energy (`telem_tx_energy_mJ_x100`). |
| `PixhawkDataBeacon data`| 49 Bytes | Compressed core flight and navigation metrics (detailed below). |

### Mathematical Data Downscaling in `PixhawkDataBeacon`
To stay within the 49-byte payload envelope without sacrificing flight telemetry resolution, raw float variables are scaled down to packed integer structures:
* **Attitude Axes:** Floating-point `roll`, `pitch`, and `yaw` (4-byte radian floats in MAVLink) are compressed into `int16_t` centi-degrees (`roll_cd`, `pitch_cd`, `yaw_cd`). The GCS upscales these back to standard radian floats natively.
* **Navigation Scaling:** Relative altitude is scaled down to decimeters (`relative_alt_dm`) inside an `int16_t` container. Both `airspeed_cms` and `groundspeed_cms` are handled as `uint16_t` in centimeters per second.
* **Compact GPS Sub-Struct (`GpsDataRingkas` - 8 Bytes):** Truncates redundant time-usec flags, reserving memory strictly for `fix_type`, `satellites_visible`, decimeter altitude (`alt_dm`), `eph`, and `epv`.

```cpp
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
}; // Bounded validation
static_assert(sizeof(TelemetryBeaconPacket) == 78, "TelemetryBeaconPacket must be exactly 78 bytes");

```

---

## State Machine & Bandwidth Throttling

The firmware implements an aggressive, SF-aware state machine to manage link degradation and bandwidth allocations.

### 1. Parameter Sync Blocking (`BLOCK_FULL_PARAM_SYNC_HIGH_SF`)

Full parameter list downloads are strictly blocked when the RF link drops to **SF10, SF11, or SF12**. At these profiles, LoRa bandwidth is mathematically incapable of rendering hundreds of Ardupilot parameters without starving the main telemetry loop. Parameter list reads are restricted to optimal configurations (**SF7 to SF9**), while discrete `PARAM_SET` hooks remain open for safety overhead.

### 2. High-Priority Command Channel (`LINK_MODE_COMMAND`)

Executing flight-critical maneuvers (such as `ARM/DISARM` or flight mode alterations via `SET_MODE`) forces the state machine into `LINK_MODE_COMMAND` for a deterministic `FLIGHT_COMMAND_HOLD_MS` window (8000 ms). Commands bypass regular transmission queues entirely via zero-wait packing pipelines (`CompactArmDisarmPacket` / `CompactSetModePacket`) ensuring instant vehicle response even during intense telemetry polling.

---

## Security Layer & Anti-Replay Engine

Because standard LoRa broadcasts are vulnerable to third-party injection, the application layer injects lightweight cryptographic validation into the telemetry pipeline:

* **Obfuscated Pre-Shared Secret:** The master authentication token `0xA56C93D1UL` is scrambled against a static obfuscation key `SECURITY_KEY_MIX` (`0x3D7F21B9UL`) rather than transmitted as plain text.
* **Per-Packet Verification:** The resultant security signature is processed on every single outgoing frame, appending a cryptographic verification tag directly into the `PacketHeader`.
* **Anti-Replay Mechanism:** Inbound payloads are verified sequentially using a strict packet window algorithm (`SECURITY_MAX_COUNTER_GAP` set to 5000). Out-of-bounds frame indexes or stale counters are immediately dropped by the hardware interrupt handler.

---

## Mission Planner Edge-Case Workarounds

Several protocol manipulations are injected directly into the ESP32 processing pipeline to guarantee Ground Control Station UI stability:

* **Telemetry ACK Decimation:** Setting `#define TELEMETRY_ACK_DECIMATION_ENABLE 1` filters down redundant GCS responses, effectively neutralizing packet collisions on the half-duplex RF boundary.
* **Dual-Component ID Shielding:** Resolves communication loop errors caused when Mission Planner handles multiple system IDs simultaneously, stopping false packet loss indicators on the HUD.
* **Autonomous Pixhawk UART Recovery:** If the serial interface connecting the ESP32 to the flight controller goes completely quiet (`PIXHAWK_UART_SILENT_MS 12000UL`), the chip runs a hot-reset sequence on the hardware serial driver without taking down the active LoRa radio link.

---

## Hardware Pinout Configuration (ESP32 + SX1278)

SPI bus assignments are mapped statically to the ESP32 `VSPI` hardware block using the following configurations:

* **MOSI:** GPIO 23
* **MISO:** GPIO 19
* **SCLK:** GPIO 18
* **NSS / Slave Select (SS):** GPIO 5
* **Reset (RST):** GPIO 25
* **DIO0 (Interrupt):** GPIO 26
* **DIO1:** `RADIOLIB_NC` (Not Connected — relies on internal status register polling via RadioLib)

### RF Transceiver Configurations:

* **Carrier Frequency:** 433.0 MHz
* **Signal Bandwidth:** 500.0 kHz
* **Coding Rate:** 4/8
* **Sync Word:** `0x12`
