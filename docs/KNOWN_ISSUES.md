# Known Issues And Validation Queue

## Current Open Items

- Arduino compile has not been run in this environment because no Arduino CLI/PlatformIO executable is available in PATH. Run both sketches after installing ESP32, RadioLib, and MAVLink dependencies.
- SF7 Turbo, SF8 Standard, and SF9 Long range need Mission Planner bench tests for parameter sync completion, command latency, and telemetry stability.
- SF7 full parameter sync may still exceed the 2 minute target until measured after the retry/window policy change. Capture parameter count, retry count, and RF metrics before changing payload layout.
- SF10-SF12 need a negative test: Mission Planner should attempt parameter get traffic and the bridge should block it while telemetry continues.
- The project is not currently a Git repository, so changes cannot be compared against a commit baseline.
- Common headers are duplicated into each sketch `src/common` folder for Arduino compatibility. `TelemetryProtoFix.h`, `LinkProfile.h`, and `BridgeMavlinkPolicy.h` must remain byte-identical between the GCS and UAV sketch folders.
- Battery telemetry can still appear missing if the flight controller reports MAVLink unknown sentinels because ArduPilot battery monitor/PDB configuration is invalid. Verify direct USB Mission Planner battery status before bridge debugging.
- Accel and compass calibration remain hardware-validation items. The source prioritizes the relevant command/status/progress traffic, but Mission Planner UI behavior cannot be certified without bench logs.

## Fixed In This Cleanup

- Fixed GCS SF scan loop after short locked-link gaps by delaying locked recovery scanning until the Mission Planner link-loss window has actually elapsed.
- Fixed GCS compile failure caused by a `MAVLINK_COMM_2` macro fallback colliding with MAVLink's channel enum.
- Removed all `clean_` filenames.
- Split GCS and UAV sketches into separate folders so both `setup()` / `loop()` definitions are not in one Arduino sketch folder.
- Centralized shared protocol and link policy headers.
- Centralized shared MAVLink command/setup policy in `BridgeMavlinkPolicy.h`.
- Removed redundant sketch-local command/link-mode/protocol constants that duplicated common headers.
- Removed the unused third shared-header copy under `firmware/common`.
- Closed high-SF get-parameter leakage on both GCS and UAV sides.
- Fixed Arduino include path layout by using sketch-local `src/common` headers.
- Fixed battery handling so unknown MAVLink battery fields do not erase valid values.
- Fixed GCS raw `BATTERY_STATUS` re-encoding.
- Reduced Compass/Calibration menu contention by not treating setup parameter reads as full parameter sync and by avoiding repeated stream reconfiguration.
- Centralized SF-specific parameter retry and bulk fill-window policy in `LinkProfile.h`.
- Removed stale duplicate parameter-bulk grace constants and unused parameter-sync timeout macros from the sketches.

## Diagnostics To Capture Next

- GCS serial metrics during SF7 full parameter sync:
  - parameter count reported by Mission Planner/ArduPilot
  - total sync duration
  - `asyncParamRequestRetryCount`
  - `paramBulkTxCount`
  - `paramBulkAckCount`
  - `paramBulkFailCount`
  - `paramValueDrop`
- Mission Planner messages at SF10-SF12 when parameter get is attempted.
- RSSI/SNR/PDR windows for each SF mode.
- Actual command-to-ACK latency for ARM/DISARM and SET_MODE.
- Compass calibration progress continuity: `MAG_CAL_PROGRESS` rate, `MAG_CAL_REPORT`, and final reboot prompt in Mission Planner.
- Accel calibration continuity: Mission Planner prompt sequence, real `COMMAND_ACK`, relevant `STATUSTEXT`, and absence of duplicate step advancement.
- Battery: direct USB `SYS_STATUS` / `BATTERY_STATUS` validity before and after LoRa bridge insertion.
- GCS reconnect-loop regression test: confirm `recoveryScanCount` does not increase during normal SF7 connect, parameter sync, or setup traffic unless packets are absent beyond the configured link-loss window.
