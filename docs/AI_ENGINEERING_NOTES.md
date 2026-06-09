# AI Engineering Notes

Purpose: keep future AI-assisted edits grounded in source code and protocol behavior.

## Rules For Future Edits

- Read the local source before proposing changes.
- Do not claim hardware validation unless a build log, serial log, or field log is present.
- Do not infer MAVLink behavior from UI symptoms alone. Check message semantics first.
- Keep GCS, UAV, and common protocol changes symmetric when they affect OTA packets.
- Do not change packed struct layout without updating both sketches and verifying static assertions.
- Treat SF10-SF12 as monitoring-only unless a later test record proves a narrower exception is safe.

## Facts Verified In This Pass

- All old `clean_` source filenames were removed.
- GCS and UAV sketches now live in separate Arduino sketch folders.
- Shared OTA protocol, link policy, and MAVLink setup policy headers now exist only in the two Arduino-local `src/common` folders.
- MAVLink setup command IDs and setup parameter prefixes are centralized in `BridgeMavlinkPolicy.h`; do not re-add separate GCS/UAV copies.
- Link mode IDs are centralized in `LinkProfile.h`; do not re-add sketch-local `LINK_MODE_*` blocks.
- High-SF get-parameter traffic is blocked at both the GCS entry point and UAV receive path.
- Battery telemetry must preserve MAVLink unknown sentinels instead of replacing valid values with unknown values.
- Compass calibration depends on `MAV_CMD_DO_START_MAG_CAL`, `MAV_CMD_DO_ACCEPT_MAG_CAL`, `MAV_CMD_DO_CANCEL_MAG_CAL`, `MAG_CAL_PROGRESS`, and `MAG_CAL_REPORT`.
- Accelerometer calibration uses `MAV_CMD_PREFLIGHT_CALIBRATION` and `MAV_CMD_ACCELCAL_VEHICLE_POS`; do not blindly repeat vehicle-position commands because duplicate position responses may advance a calibration step incorrectly.
- Parameter-sync retry timing, bulk fill grace, and bounded bulk fill-window policy are centralized in `LinkProfile.h`.
- The duplicated `src/common` folders are build-layout duplication only. Treat them as mirrored files; do not edit one side without applying the same change to the other side.

## Documentation Anchors

- MAVLink parameter get operations produce `PARAM_VALUE` / `PARAM_EXT_VALUE` responses.
- MAVLink command operations depend on `COMMAND_ACK`.
- ArduPilot GCS failsafe depends on MAVLink heartbeat loss timing.
- RadioLib SX1278 supports LoRa SF configuration and requires matching RF settings across nodes.

Official references:

- https://mavlink.io/en/services/parameter.html
- https://mavlink.io/en/services/command.html
- https://ardupilot.org/planner/docs/common-accelerometer-calibration.html
- https://ardupilot.org/copter/docs/gcs-failsafe.html
- https://jgromes.github.io/RadioLib/class_s_x1278.html
- https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1276
