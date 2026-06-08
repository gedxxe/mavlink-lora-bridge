# MAVLink-LoRa Bridge For ArduPilot Telemetry

This repository contains two Arduino sketches for an ESP32 + SX1278 half-duplex LoRa bridge between Mission Planner and an ArduPilot flight controller.

- GCS node: receives LoRa packets from the UAV node, reconstructs selected MAVLink messages, and exposes a MAVLink serial stream to Mission Planner.
- UAV node: parses MAVLink from Pixhawk/ArduPilot, compresses selected real-time telemetry into fixed LoRa packets, and forwards selected Mission Planner commands to the flight controller.

The bridge is MAVLink-aware by design. It is not a transparent serial modem. This is required because MAVLink parameter, mission, log, and calibration traffic can exceed the airtime budget of a half-duplex LoRa link, especially at high spreading factors.

No source file uses the old `clean_` prefix.

# Changelog

## 2026-06-09

### Changed

- Removed the `clean_` prefix from all source filenames.
- Split the project into two Arduino sketch folders:
  - `firmware/GCS_TurboFast`
  - `firmware/UAV_TurboFast`
- Kept shared protocol, link, and MAVLink setup policy headers inside each sketch `src/common` folder for Arduino IDE compatibility.
- Updated include paths to use the new structure.
- Replaced stale README content with current build, structure, and operating policy documentation.
- Added `BridgeMavlinkPolicy.h` to centralize MAVLink compatibility fallbacks, ArduPilot setup command IDs, and setup parameter classifiers.
- Moved `LINK_MODE_*` definitions into `LinkProfile.h`.
- Removed redundant local packet-type, LoRa PHY CRC/header, compact-command length, MAVLink command, and link-mode definitions from the sketches.
- Replaced duplicated setup parameter and calibration command classifier bodies with shared policy calls.
- Moved async parameter retry timing and parameter-bulk fill-window policy into `LinkProfile.h`.
- Updated SF7 Turbo parameter sync policy to use shorter Pixhawk-response retry and a larger bounded bulk prefetch window.

### Fixed

- Fixed GCS compile failure caused by `BridgeMavlinkPolicy.h` defining `MAVLINK_COMM_2` before MAVLink's own `mavlink_channel_t` enum was parsed. The policy header now includes `MAVLink_ardupilotmega.h` first and no longer defines `MAVLINK_COMM_*` macro fallbacks.
- Fixed Arduino IDE include failure by changing sketch includes to local `src/common/...` paths and adding matching `src/common` header copies to both sketch folders.
- Fixed source/include mismatch where sketches included names such as `TelemetryProtoFix.h` but the files were still named `clean_TelemetryProtoFix.h`.
- Closed the high-SF get-parameter leak:
  - GCS now blocks parameter get traffic before a valid beacon lock.
  - GCS blocks all parameter get requests at SF10-SF12.
  - UAV blocks all parameter get requests at SF10-SF12.
  - UAV drops stale/non-write-ack `PARAM_VALUE` / `PARAM_EXT_VALUE` leakage at SF10-SF12.
- Fixed battery telemetry merging so MAVLink unknown values do not overwrite the last valid voltage/current/remaining percentage.
- Forwarded raw `BATTERY_STATUS` through the GCS re-encoder when it is received from the UAV side.
- Prevented Mission Planner compass/setup parameter reads from starting full parameter sync.
- Reduced repeated calibration-mode blocking by running Pixhawk stream interval reconfiguration only on calibration-mode entry.
- Removed unused local `RADIOLIB_ERR_UNKNOWN`, `paramIdStartsWith()`, and `paramIdEquals()` definitions.
- Removed the unused third shared-header copy under `firmware/common`; GCS and UAV now retain only the Arduino-required `src/common` copies.
- Removed stale sketch-local `PARAM_BULK_FILL_GRACE_SF*` definitions that duplicated obsolete SF timing values.
- Removed stale sketch-local `PARAM_SYNC_NO_VALUE_EXIT_MS_*`, `PARAM_SYNC_LINK_STALL_MS_*`, and unused telemetry ACK-timeout macros that were already superseded by `LinkProfile.h` functions.
- Added explicit prototypes for new UAV parameter-sync policy helpers so the sketch is less dependent on Arduino auto-prototype generation.

### Not Yet Validated On Hardware

- Arduino compile was not yet confirmed in this cleanup pass.
- Mission Planner behavior at SF7, SF8, and SF9 still needs bench validation with real GCS/UAV hardware.
- SF10-SF12 monitoring mode needs runtime confirmation that telemetry cadence remains stable while Mission Planner attempts auto parameter reads.
- Compass calibration needs bench validation that `MAG_CAL_PROGRESS`, `MAG_CAL_REPORT`, `COMMAND_ACK`, and setup parameter reads update Mission Planner continuously.
- Accel, level, and simple accel calibration need bench validation that Mission Planner receives real ACK/status and advances each prompt once.
- SF7 two-minute parameter-sync target is not proven by static audit.


## Repository Structure

```text
mavlink-lora-bridge/
|-- firmware/
|   |-- GCS_TurboFast/
|   |   |-- GCS_TurboFast.ino
|   |   |-- BeaconDecodeHelpers.h
|   |   |-- GCCalibHelpers.h
|   |   |-- GCSCommandQueue.h
|   |   |-- GCSRadioHelpers.h
|   |   `-- src/common/
|   |       |-- BridgeMavlinkPolicy.h
|   |       |-- LinkProfile.h
|   |       `-- TelemetryProtoFix.h
|   `-- UAV_TurboFast/
|       |-- UAV_TurboFast.ino
|       |-- BeaconFillHelpers.h
|       |-- UAVCalibHelpers.h
|       |-- UAVParamQueue.h
|       |-- UAVRadioHelpers.h
|       `-- src/common/
|           |-- BridgeMavlinkPolicy.h
|           |-- LinkProfile.h
|           `-- TelemetryProtoFix.h
|-- docs/
|   |-- CONFIGURATION.md
|   |-- KNOWN_ISSUES.md
|   `-- SF_LINK_POLICY.md
|-- LICENSE
`-- README.md
```

The two `src/common` folders are intentionally duplicated because Arduino IDE builds each sketch from its own sketch directory. Parent-directory includes were already shown to be fragile in this project. The removed `firmware/common` folder was a third copy that was not used by either sketch build path.

The two common header sets must remain byte-identical:

- `TelemetryProtoFix.h`: fixed OTA packet structs, CRC/auth helpers, LoRa airtime helpers, compact command structs, and packet constants.
- `LinkProfile.h`: SF policy, link-mode IDs, and SF-dependent timing helpers, including parameter retry timing and bulk fill-window policy.
- `BridgeMavlinkPolicy.h`: MAVLink compatibility fallbacks, ArduPilot setup command aliases, setup parameter classifier, calibration command classifier, and flight-action command classifier.

## Build Environment

Compile each sketch independently:

1. Open `firmware/GCS_TurboFast/GCS_TurboFast.ino`.
2. Open `firmware/UAV_TurboFast/UAV_TurboFast.ino`.
3. Install the required Arduino dependencies:
   - ESP32 Arduino core.
   - RadioLib with SX1278 support.
   - MAVLink C headers that provide `MAVLink_ardupilotmega.h`.
4. Confirm both nodes use matching RF constants:
   - `FREQ_MHZ`
   - `LORA_BW_KHZ`
   - `LORA_CR_DEN`
   - `LORA_SYNC`
   - `SF_MIN`, `SF_MAX`
   - `TP_MIN`, `TP_MAX`

The current source uses `FREQ_MHZ 420.0`. Verify local spectrum allocation, output power limits, antenna gain, and duty-cycle rules before radiated testing.

Arduino-local includes must remain in this form:

```cpp
#include "src/common/TelemetryProtoFix.h"
#include "src/common/LinkProfile.h"
#include "src/common/BridgeMavlinkPolicy.h"
```

Do not use absolute includes such as `/common/TelemetryProtoFix.h`, and do not rely on parent-directory includes such as `../common/...`.

## System Model

The link is modeled as a half-duplex LoRa channel carrying multiple MAVLink-derived traffic classes:

- compact periodic telemetry beacons;
- high-priority raw MAVLink command/ACK/status packets;
- compact command packets;
- bounded parameter bulk packets;
- configuration proposal/acknowledgement packets.

The bridge schedules these classes because LoRa time-on-air grows rapidly with spreading factor. A MAVLink parameter download that is acceptable on USB or SiK radio can starve flight command feedback and telemetry freshness on SF10-SF12.

## Fundamental LoRa Timing

Let:

- `SF` be spreading factor, normally 7 to 12 on SX1278 LoRa.
- `BW` be bandwidth in Hz.
- `CR` be the coding-rate denominator used in the Semtech payload-symbol expression, commonly 5 to 8 for 4/5 to 4/8.
- `PL` be payload length in bytes.
- `CRC` be 1 when payload CRC is enabled, otherwise 0.
- `IH` be 1 for implicit header, 0 for explicit header.
- `DE` be low-data-rate optimization, typically enabled when symbol duration is long.
- `Npreamble` be configured preamble length in symbols.

Symbol duration:

```text
T_sym = (2^SF) / BW
```

Preamble duration:

```text
T_preamble = (Npreamble + 4.25) * T_sym
```

Payload symbol count, following the Semtech SX1276/77/78/79 LoRa packet model:

```text
N_payload = 8 + max(
  ceil((8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE))) * CR,
  0
)
```

Payload duration:

```text
T_payload = N_payload * T_sym
```

Packet time-on-air:

```text
T_packet = T_preamble + T_payload
```

The important engineering consequence is exponential airtime growth with `SF`, because `T_sym` is proportional to `2^SF`. For a fixed packet size and bandwidth, moving from SF7 to SF12 increases symbol duration by a factor of:

```text
2^(12 - 7) = 32
```

This does not mean every packet is exactly 32 times longer because payload symbol count also changes, but it correctly captures the dominant scaling.

Nominal LoRa physical-layer bit rate can be approximated as:

```text
R_b = SF * BW * (4 / CR) / (2^SF)
```

This is a PHY approximation. It does not include preamble, header, CRC, half-duplex guard time, retransmission, or queueing delay.

## Airtime Utilization And Scheduling

For a periodic traffic class `i`, define:

- `T_i` as packet airtime.
- `G_i` as guard/listen/turnaround overhead.
- `P_i` as period.

Approximate channel utilization:

```text
U = sum((T_i + G_i) / P_i)
```

The link should be operated with margin:

```text
U < U_max
```

where `U_max` must be below 1.0 for a real system because LoRa reception windows, MCU scheduling jitter, retransmission, Mission Planner retries, and ArduPilot stream bursts consume residual airtime. The exact margin must be validated on hardware; it cannot be proven from source code alone.

For packet delivery ratio:

```text
PDR = N_rx_valid / N_tx
```

For one-way or command-to-feedback latency:

```text
L = t_feedback_received - t_command_sent
```

For transmit energy:

```text
E_tx = V_supply * I_tx(TP) * T_packet
```

where `I_tx(TP)` depends on the module, PA path, supply voltage, board layout, and configured transmit power. The helper in `TelemetryProtoFix.h` is an estimate, not a substitute for current measurement.

## MAVLink Traffic Implications

MAVLink parameter get operations are bulk operations:

- `PARAM_REQUEST_LIST` causes the target component to emit all parameters as `PARAM_VALUE`.
- `PARAM_REQUEST_READ` causes one `PARAM_VALUE`.
- `PARAM_SET` expects a `PARAM_VALUE` acknowledgement after the set attempt.
- Extended parameters follow the same request/response pattern with `PARAM_EXT_*`.

If the number of parameters is `N_param` and the average encoded response packet airtime is `T_param`, the lower-bound airtime for a full parameter list is:

```text
T_param_list >= N_param * T_param
```

This lower bound excludes half-duplex ACK slots, retries, queueing delay, Mission Planner gap-fill reads, and ArduPilot pacing. Therefore full parameter sync is allowed only on SF7-SF9 in this firmware.

The implemented SF policy is:

| Mode | SF | Intended use | Parameter get policy |
| --- | --- | --- | --- |
| Turbo | SF7 | Fast setup, parameter sync, bench testing | Allowed |
| Standard | SF8 | Normal setup and telemetry | Allowed |
| Long range | SF9 | Slower setup with higher link margin | Allowed |
| Monitoring | SF10-SF12 | Critical telemetry and flight commands | Blocked |

At SF10-SF12, get-parameter traffic is blocked at both the GCS entry point and the UAV receive path. `PARAM_SET` remains allowed because MAVLink uses `PARAM_VALUE` as the acknowledgement for a write operation.

## Parameter Sync Implementation

The UAV node proxies a Mission Planner full parameter sync as indexed `PARAM_REQUEST_READ` requests to ArduPilot rather than forwarding `PARAM_REQUEST_LIST` transparently. The proxy keeps one outstanding Pixhawk request per parameter index and packs received `PARAM_VALUE` records into bounded `PARAM_BULK` LoRa packets. This is a reliability tradeoff: it avoids unbounded UART-to-LoRa burst amplification while allowing the LoRa transmit queue to stay filled.

If:

- `N_param` is the parameter count reported by ArduPilot;
- `R_bulk(SF)` is the maximum records packed in one `PARAM_BULK`;
- `T_bulk(SF, PL)` is the LoRa airtime of a packed bulk packet;
- `T_ack(SF)` is the acknowledgement/listen slot time;
- `N_retry` is the number of RF or Pixhawk request retries;

then a practical lower-bound model is:

```text
N_bulk >= ceil(N_param / R_bulk(SF))
T_sync >= N_bulk * (T_bulk(SF, PL) + T_ack(SF)) + N_retry * T_retry(SF)
```

This is still optimistic because it excludes Mission Planner gap-fill reads, ArduPilot scheduling jitter, serial buffering, and RF retransmission. The current SF7 optimization keeps the RF payload format unchanged and only changes the profile policy:

- `linkParamRequestRetryMs(SF7) = 650 ms`;
- `linkParamBulkFillWindow(SF7) = 5 queued bulk packets`;
- `linkParamBulkFillGraceMs(SF7) = 80 ms`;
- `linkParamBulkRecords(SF7) = PARAM_BULK_MAX_RECORDS`.

SF8 and SF9 remain allowed but intentionally slower because airtime increases with SF. SF10-SF12 remain monitoring-only and are not valid modes for full Mission Planner parameter download.

## Battery Telemetry Policy

The firmware merges battery data from `SYS_STATUS` and `BATTERY_STATUS`.

MAVLink unknown sentinels must be preserved:

- `SYS_STATUS.voltage_battery == UINT16_MAX`: voltage unknown.
- `SYS_STATUS.current_battery == -1`: current unknown.
- `SYS_STATUS.battery_remaining == -1`: percentage unknown.

The UAV node updates compact battery fields only when an incoming value is valid. This prevents an unknown field from erasing the last valid measurement. Raw `BATTERY_STATUS` can also be forwarded to Mission Planner on SF7-SF9 when raw forwarding policy permits it.

If Mission Planner still shows no voltage, current, or remaining percentage, the first diagnostic boundary is the flight controller itself: verify over USB that ArduPilot emits valid `SYS_STATUS` or `BATTERY_STATUS` values from the configured battery monitor. The bridge cannot synthesize a valid battery estimate from MAVLink unknown sentinels without introducing false telemetry.

## Compass Calibration Policy

Mission Planner compass calibration depends on ArduPilot command and progress traffic:

- `MAV_CMD_DO_START_MAG_CAL`
- `MAV_CMD_DO_ACCEPT_MAG_CAL`
- `MAV_CMD_DO_CANCEL_MAG_CAL`
- `COMMAND_ACK`
- `STATUSTEXT`
- `MAG_CAL_PROGRESS`
- `MAG_CAL_REPORT`

The bridge treats setup commands and setup parameter reads as interactive control/configuration traffic. Setup parameter reads open calibration/config mode but no longer start full parameter sync. This avoids a known collision where Mission Planner setup-page polling could compete with the full parameter-sync state machine.

SF10-SF12 remain monitoring modes. Full Mission Planner setup and parameter-heavy compass workflows should be tested and used at SF7-SF9 or over USB.

Accelerometer calibration is handled as a setup command path, not as telemetry. `MAV_CMD_PREFLIGHT_CALIBRATION` and `MAV_CMD_ACCELCAL_VEHICLE_POS` are classified as interactive setup traffic so their real `COMMAND_ACK` / `STATUSTEXT` feedback can bypass bulk queues. The bridge does not blindly repeat `MAV_CMD_ACCELCAL_VEHICLE_POS` because that command is also used to advance/report vehicle positions during accelerometer calibration; duplicating it can advance a calibration step incorrectly.

## Protocol Invariants

These invariants protect over-the-air compatibility:

- `TelemetryBeaconPacket` size remains fixed at 109 bytes.
- `PixhawkDataBeacon` size remains fixed at 80 bytes.
- OTA structs in `TelemetryProtoFix.h` use packed layout.
- Packet CRC and auth tag are computed over the fixed packet byte layout.
- GCS and UAV `src/common` headers must remain byte-identical.
- Any struct layout change requires a packet-size audit on both sketches.

## Redundancy Cleanup Status

Removed in the current cleanup pass:

- the unused third shared-header copy under `firmware/common`;
- sketch-local duplicate MAVLink command constants;
- sketch-local duplicate link-mode constants;
- sketch-local duplicate packet type constants;
- sketch-local duplicate LoRa PHY CRC/header constants;
- sketch-local duplicate compact-command length macro;
- unused local `RADIOLIB_ERR_UNKNOWN`;
- unused local `paramIdStartsWith()` and `paramIdEquals()` wrappers.
- stale sketch-local `PARAM_BULK_FILL_GRACE_SF*` constants;
- sketch-local async parameter retry and bulk fill-window constants now represented by `LinkProfile.h`.

Retained intentionally:

- one `src/common` header set under GCS;
- one `src/common` header set under UAV.

This retained duplication is an Arduino build-layout constraint, not an independent protocol fork. Static checks compare the two header sets by hash.

## Validation Status

Static checks performed in this environment:

- quoted relative includes resolve;
- GCS and UAV `src/common` headers are byte-identical;
- GCS and UAV sketch preprocessor conditional stacks are balanced;
- no `clean_` source files remain;
- no targeted duplicate command/link/protocol macro definitions remain in the main sketches.
- `LinkProfile.h` policy copies were kept byte-identical after the SF7 parameter-sync optimization.

Not yet performed in this environment:

- Arduino compile, because `arduino-cli`, `arduino`, and `pio` are not available in PATH;
- Mission Planner bench test at SF7/SF8/SF9;
- high-SF negative test proving parameter get is blocked while telemetry remains live;
- hardware measurement of command-to-ACK latency and PDR.

## Current Problem Audit Status

The following conclusions are based on source-level audit only, not a bench log:

| Reported problem | Code-level status after this pass |
| --- | --- |
| SF7 full parameter sync takes about 3 min 50 s; target is 2 min | Still potentially exists until hardware timing is measured. SF7 retry and bulk fill-window policy were optimized without changing OTA structs. |
| SF8-SF9 parameter sync is much slower | Expected from LoRa airtime scaling; still allowed, but not expected to match SF7. |
| Normal telemetry works | Code path remains intact; compact telemetry is still the primary normal-mode traffic. |
| Accel/Level/Simple Accel calibration fails | Potentially exists until Mission Planner bench validation. Setup commands and accel calibration vehicle-position command are prioritized and not duplicated blindly. |
| Compass calibration progress/IDs fail to update | Potentially exists until bench validation. `MAG_CAL_PROGRESS`, `MAG_CAL_REPORT`, `COMMAND_ACK`, `STATUSTEXT`, and setup parameter reads are prioritized at SF7-SF9. |
| Battery voltage/current/remaining not shown | Bridge-side sentinel handling is fixed. Remaining risk is upstream ArduPilot battery monitor configuration or invalid MAVLink values from the FC/PDB path. |

## Recommended Bench Test Matrix

1. Compile both sketches with ESP32 core, RadioLib, and MAVLink ArduPilotMega headers.
2. Verify both nodes boot with identical RF constants.
3. At SF7, perform full Mission Planner parameter sync and log:
   - parameter count;
   - sync duration;
   - `asyncParamRequestRetryCount`;
   - `paramBulkTxCount`;
   - `paramBulkAckCount`;
   - `paramBulkFailCount`;
   - `paramValueDrop`.
4. Repeat setup/parameter tests at SF8 and SF9.
5. At SF10-SF12, attempt parameter get traffic and verify:
   - operator receives a block notice;
   - compact telemetry continues;
   - flight commands still receive real feedback.
6. Run compass calibration at SF7-SF9 and verify continuous `MAG_CAL_PROGRESS`, final `MAG_CAL_REPORT`, `COMMAND_ACK`, and Mission Planner UI update.
7. Run full accel, level, and simple accel calibration at SF7-SF9 and verify real `COMMAND_ACK`, relevant `STATUSTEXT`, and Mission Planner step progression.
8. Verify battery over USB first, then through LoRa:
   - USB direct from FC to Mission Planner must show valid battery;
   - LoRa bridge must preserve the same voltage/current/remaining semantics;
   - if USB is invalid, correct ArduPilot battery monitor/PDB configuration before bridge debugging.
9. Measure RSSI, SNR, PDR, and command-to-feedback latency for each SF.

# Audit Report

## 2026-06-09

### Include And Build Layout

Finding: Arduino IDE failed on `/common/TelemetryProtoFix.h` because parent or absolute include paths are not robust from a sketch build folder.

Action:

- GCS and UAV now include `src/common/TelemetryProtoFix.h` and `src/common/LinkProfile.h`.
- GCS and UAV now include `src/common/BridgeMavlinkPolicy.h` for shared MAVLink command/setup policy.
- Matching `src/common` copies were added under both sketch folders.
- Static include resolution was checked.

Follow-up finding: GCS compile failed when `BridgeMavlinkPolicy.h` was included before MAVLink headers. The policy header defined `MAVLINK_COMM_2` as a macro, but `MAVLINK_COMM_2` is an enum member in MAVLink's `mavlink_channel_t`. That macro expansion corrupted `mavlink_types.h` and then cascaded into `MAV_CMD_*` enum parse errors.

Action:

- `BridgeMavlinkPolicy.h` now includes `MAVLink_ardupilotmega.h` before compatibility aliases.
- Removed the `MAVLINK_COMM_2` macro fallback from both GCS and UAV policy headers.
- Kept GCS and UAV `BridgeMavlinkPolicy.h` copies byte-identical.

### Redundancy And Structure

Finding: GCS and UAV carried duplicate MAVLink command IDs, setup parameter prefix classifiers, calibration command classifiers, link-mode constants, packet type constants, and unused helper wrappers.

Action:

- Added `BridgeMavlinkPolicy.h` as the single source for MAVLink compatibility fallbacks, ArduPilot setup command aliases, setup parameter prefix classification, setup command classification, and flight-action command classification.
- Moved `LINK_MODE_*` constants into `LinkProfile.h`.
- Removed sketch-local redefinitions for `PKT_*`, `LORA_PHY_CRC_ENABLED`, `LORA_IMPLICIT_HEADER`, `COMPACT_CMD_MAX_LEN`, MAVLink command fallbacks, and link modes.
- Removed unused `RADIOLIB_ERR_UNKNOWN`, `paramIdStartsWith()`, and `paramIdEquals()` sketch definitions.

Remaining limitation:

- `TelemetryProtoFix.h`, `LinkProfile.h`, and `BridgeMavlinkPolicy.h` still exist in two physical locations, one per Arduino sketch. This is intentional for Arduino IDE compatibility; byte identity is verified statically. The unused third copy under `firmware/common` was removed.

### Battery Telemetry

Finding: `SYS_STATUS` and `BATTERY_STATUS` use explicit unknown sentinels. The old UAV parser copied `SYS_STATUS` fields directly, so `UINT16_MAX` voltage or `-1` current/remaining could overwrite previously valid telemetry.

Action:

- Added battery validity checks and merge logic on the UAV side.
- Initialized battery fields to MAVLink unknown sentinel values.
- Added GCS raw `BATTERY_STATUS` re-encoding so Mission Planner can receive the richer battery message when raw forwarding is available.

Remaining limitation:

- The compact beacon still carries only aggregate battery voltage/current/remaining via the SYS_STATUS-compatible fields. Per-cell detail is available only when raw `BATTERY_STATUS` is forwarded.

### Compass Calibration

Finding: Setup parameter reads were treated like full parameter sync, and calibration stream interval configuration could run repeatedly while Mission Planner polled setup pages.

Action:

- Setup parameter reads now enter calibration/config mode without starting full parameter sync.
- UAV stream interval reconfiguration for calibration now runs only when entering calibration mode.
- Existing high-priority forwarding for `COMMAND_ACK`, `STATUSTEXT`, `MAG_CAL_PROGRESS`, and `MAG_CAL_REPORT` remains in place.

Remaining limitation:

- SF10-SF12 intentionally remain monitoring/command modes and block get-parameter traffic. Full Mission Planner compass setup and parameter-heavy calibration should be tested at SF7-SF9.

### Scheduling And Blocking

Finding: The firmware still uses blocking RadioLib TX/RX calls, which is expected for the current half-duplex design. The highest-risk avoidable blocking was repeated stream reconfiguration during setup/calibration, now reduced.

Remaining risks:

- `radio.transmit()` and bounded `radio.receive(..., timeout)` still block the loop during RF slots.
- Stream configuration functions still contain small `delay(8)` gaps, but they are now limited to mode transitions and setup/param transitions, not every calibration poll.
- Hardware validation is still required for timing under SF7/SF8/SF9 and for high-SF negative tests.

### SF7-SF9 Parameter Sync Follow-Up

Finding: Source audit found stale SF-specific partial-bulk grace constants in `UAV_TurboFast.ino` that no longer controlled behavior because `paramBulkFillGraceForSF()` already delegates to `LinkProfile.h`. The async parameter retry interval and bulk prefetch window were also sketch-local rather than part of the shared SF policy.

Action:

- Removed stale `PARAM_BULK_FILL_GRACE_SF*` definitions from the UAV sketch.
- Removed stale parameter-sync timeout macros that were no longer referenced after timeout policy moved to `LinkProfile.h`.
- Added `linkParamRequestRetryMs()` to `LinkProfile.h`.
- Added `linkParamBulkFillWindow()` to `LinkProfile.h`.
- Updated the UAV async parameter poller to use the SF policy functions.
- Kept OTA `PARAM_BULK` packet layout unchanged.

Assessment:

- SF7 full parameter sync was still a plausible bottleneck before this patch because a 1500 ms Pixhawk-response retry could create long stalls after a missed `PARAM_VALUE`.
- The patch reduces retry delay at SF7 and allows a larger bounded RF-side prefetch window so the LoRa transmitter is less likely to idle while the Pixhawk UART is producing parameter values.
- The reported 2 minute SF7 target cannot be claimed as met until measured with the actual parameter count, RSSI/SNR, retry count, and Mission Planner gap-fill behavior.
- SF8 and SF9 remain expected to be slower than SF7 due to LoRa airtime scaling.

### Six-Issue Source-Level Status

| Item | Status |
| --- | --- |
| SF7 parameter sync approximately 3 min 50 s | Potentially still exists; code now has SF7 retry/window optimization, but hardware timing is required. |
| SF8-SF9 very slow parameter sync | Expected risk; allowed by policy but constrained by airtime. |
| Normal telemetry works | No code path regression found in static audit. |
| Accel/Level/Simple Accel calibration failure | Potentially still exists until bench validation; setup commands are prioritized and `CMD_ACCELCAL_VEHICLE_POS` is classified, but not duplicated blindly. |
| Compass calibration loading/ID update failure | Potentially still exists until bench validation; progress/report/ACK/status and setup parameter reads are prioritized at SF7-SF9. |
| Battery voltage/current/remaining missing | Bridge-side invalid-sentinel handling is corrected; remaining risk is upstream FC/PDB/ArduPilot battery monitor configuration or invalid values emitted by the FC. |



## References

- MAVLink Parameter Protocol: https://mavlink.io/en/services/parameter.html
- MAVLink Extended Parameter Protocol: https://mavlink.io/en/services/parameter_ext.html
- MAVLink Common Message Set: https://mavlink.io/en/messages/common.html
- MAVLink ArduPilotMega Dialect: https://mavlink.io/en/messages/ardupilotmega.html
- MAVLink Command Protocol: https://mavlink.io/en/services/command.html
- ArduPilot Mission Planner Accelerometer Calibration: https://ardupilot.org/planner/docs/common-accelerometer-calibration.html
- ArduPilot Mission Planner Compass Calibration: https://ardupilot.org/planner/docs/common-compass-calibration-in-mission-planner.html
- Semtech SX1276/77/78/79 datasheet family: https://www.semtech.com/products/wireless-rf/lora-connect/sx1276
- RadioLib SX1278 API: https://jgromes.github.io/RadioLib/class_s_x1278.html
