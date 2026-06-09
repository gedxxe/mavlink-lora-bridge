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

### GCS Connect/Disconnect Loop

Finding: The GCS locked-SF recovery scan used `linkGcsIdleScanAfterMs()` directly. At SF7 this was 1400 ms, while the UAV can legitimately stretch beacon gaps during Mission Planner connect, parameter sync, or setup feedback because PARAM_BULK and ACK slots share the half-duplex LoRa channel. A single delayed beacon could make GCS leave `phyLocked`, start SF scanning, stop sending timely downlink heartbeat/ACK opportunities, and cause Mission Planner/ArduPilot failsafe symptoms.

Action:

- Added `GCS_LOCKED_SCAN_EXTRA_GRACE_MS`.
- Updated `updateRecoveryScanning()` so an already locked GCS does not scan away from the current SF until the idle gap exceeds `mpLinkLossTimeoutForSF(activeSF) + GCS_LOCKED_SCAN_EXTRA_GRACE_MS`.
- Kept boot/unlocked weighted scanning behavior unchanged.
- Kept hard radio recovery unchanged at `GCS_LORA_NO_PACKET_RECOVERY_MS`.

Assessment:

- This is a targeted state-machine fix. It does not alter OTA packet layout, MAVLink identities, RF frequency, bandwidth, coding rate, or ACK packet format.
- It should directly address the reported pattern: several dozen connected packets, then GCS scan loop and Mission Planner disconnect/failsafe.
- Hardware validation is still required by logging `recoveryScanCount`, `lastPacketMs` age, RSSI/SNR, and Mission Planner heartbeat continuity.

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
