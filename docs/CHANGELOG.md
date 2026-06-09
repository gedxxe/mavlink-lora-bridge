# Changelog

## 2026-06-09

### Changed

- Reworked README mathematical documentation for GitHub LaTeX rendering using `$$...$$` blocks, including LoRa time-on-air, utilization, PDR/PLR, latency, energy, and parameter-sync models.
- Added README architecture overview with a Mermaid data-flow diagram and layer-responsibility table for GCS, UAV, OTA protocol, MAVLink policy, telemetry, parameter sync, and calibration/setup paths.
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

- Fixed GCS connect/disconnect looping where a locked SF could be abandoned after a short idle gap. Locked recovery scanning is now aligned to the Mission Planner link-loss window plus guard time, preventing a single delayed beacon during parameter sync/setup traffic from forcing SF scanning and breaking the GCS heartbeat path.
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
