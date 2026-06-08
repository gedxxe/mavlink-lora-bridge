# SF Link Policy

This file is the engineering reminder for the LoRa spreading-factor policy. The build entry points are the matching `LinkProfile.h` files in each sketch `src/common` folder. MAVLink setup/compass command classification is centralized separately in the matching `BridgeMavlinkPolicy.h` files in each sketch `src/common` folder.

## Modes

| SF | Name | Primary purpose | Parameter get traffic |
| --- | --- | --- | --- |
| SF7 | Turbo | Fast setup and parameter sync | Allowed |
| SF8 | Standard | Normal setup, telemetry, command use | Allowed |
| SF9 | Long range | Slower setup with higher link margin | Allowed |
| SF10 | Monitoring | Critical real-time telemetry and commands | Blocked |
| SF11 | Monitoring | Critical real-time telemetry and commands | Blocked |
| SF12 | Monitoring | Critical real-time telemetry and commands | Blocked |

## Rationale

MAVLink parameter get operations are bulk traffic. `PARAM_REQUEST_LIST` causes the target component to broadcast all parameters as `PARAM_VALUE` messages. `PARAM_REQUEST_READ` causes a `PARAM_VALUE` response for one parameter. Extended parameters use the same pattern with `PARAM_EXT_*` messages.

On a half-duplex SX1278 link, high spreading factors increase symbol time and airtime. That makes bulk request/response traffic unsuitable for SF10-SF12 because it can starve flight commands, heartbeat-derived failsafe behavior, and compact telemetry beacons.

The bridge therefore treats SF10-SF12 as monitoring modes:

- pass compact telemetry beacons;
- pass flight-critical commands and their real ACK/status feedback;
- block get-parameter requests;
- allow `PARAM_SET` write acknowledgements because MAVLink defines `PARAM_VALUE` as the acknowledgement after a set operation.

## Parameter Sync Policy

Full parameter sync is proxied by the UAV as indexed `PARAM_REQUEST_READ` requests and compacted into `PARAM_BULK` packets. The current policy is centralized in both `LinkProfile.h` copies:

| Policy function | SF7 Turbo | SF8 Standard | SF9 Long range |
| --- | --- | --- | --- |
| `linkParamRequestRetryMs()` | 650 ms | 900 ms | 1200 ms |
| `linkParamBulkFillWindow()` | 5 packets | 4 packets | 3 packets |
| `linkParamBulkFillGraceMs()` | 80 ms | 130 ms | 190 ms |
| `linkParamBulkRecords()` | max records | up to 8 records | up to 6 records |

This policy does not change the LoRa packet format. It only changes how aggressively the UAV keeps the bounded bulk queue supplied while preserving one outstanding Pixhawk parameter request per index.

## Safe Field Procedure

1. Use USB or SF7-SF9 for full Mission Planner parameter sync.
2. Confirm both nodes use matching frequency, bandwidth, coding rate, sync word, and power limits.
3. For SF10-SF12, verify Mission Planner can display telemetry without relying on a fresh full parameter download.
4. Treat any repeated `Param get blocked: use SF7-SF9` message as expected behavior, not RF packet loss.
5. If command ACKs are delayed, inspect half-duplex slot timing before increasing retry rates.
6. For SF7 two-minute parameter-sync experiments, record `asyncParamRequestRetryCount`, parameter count, RSSI, SNR, and bulk ACK/fail counters before changing payload structures.

## Evidence Sources

- MAVLink parameter protocol: https://mavlink.io/en/services/parameter.html
- MAVLink command protocol: https://mavlink.io/en/services/command.html
- ArduPilot GCS failsafe: https://ardupilot.org/copter/docs/gcs-failsafe.html
- RadioLib SX1278 API: https://jgromes.github.io/RadioLib/class_s_x1278.html
- Semtech SX1276/77/78/79 family: https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1276
