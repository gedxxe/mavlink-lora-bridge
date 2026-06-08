# Configuration Map

This project still uses compile-time constants. Treat them as configuration points, not magic numbers. Keep GCS and UAV RF settings synchronized.

## Central Link Policy

Build files:

- `firmware/GCS_TurboFast/src/common/LinkProfile.h`
- `firmware/UAV_TurboFast/src/common/LinkProfile.h`

Keep both copies synchronized and byte-identical.

- `LINK_SF_TURBO`: SF7.
- `LINK_SF_NORMAL`: SF8.
- `LINK_SF_LONG_RANGE`: SF9.
- `LINK_SF_SETUP_MAX`: highest SF allowed for setup and parameter get.
- `LINK_SF_MONITORING_MIN`: first SF treated as monitoring-only.
- `LINK_MODE_*`: shared GCS/UAV link mode identifiers.
- `linkNormalTelemetryIntervalMs()`: normal beacon cadence by SF.
- `linkParamSyncTelemetryIntervalMs()`: beacon cadence during parameter sync.
- `linkParamBulkFillGraceMs()`: partial parameter-bulk release delay by SF.
- `linkParamRequestRetryMs()`: Pixhawk `PARAM_REQUEST_READ` retry delay by SF.
- `linkParamBulkFillWindow()`: bounded number of queued parameter bulk packets allowed while the async proxy continues polling Pixhawk.
- `linkParamBulkRecords()`: maximum packed parameter records by SF.
- `linkParamBulkAckTimeoutMs()`: LoRa acknowledgement timeout for parameter bulk packets.
- `linkTelemetryAckTimeoutMs()`: reduced telemetry ACK wait policy for active parameter sync.
- `linkGcsRxTimeoutMs()`: GCS receive timeout by SF.
- `linkGcsIdleScanAfterMs()`: GCS scan recovery timing by SF.

## Central MAVLink Setup Policy

Build files:

- `firmware/GCS_TurboFast/src/common/BridgeMavlinkPolicy.h`
- `firmware/UAV_TurboFast/src/common/BridgeMavlinkPolicy.h`

Keep both copies synchronized and byte-identical.

- `CMD_DO_START_MAG_CAL`, `CMD_DO_ACCEPT_MAG_CAL`, and `CMD_DO_CANCEL_MAG_CAL`: ArduPilot compass calibration commands.
- `bridgeIsInteractiveSetupParamId()`: setup/calibration parameter prefix classifier used by both sketches.
- `bridgeIsSetupConfigCommand()`: setup command classifier used to prioritize Mission Planner setup flows.
- `bridgeIsFlightActionCommand()`: command classifier used to interrupt bulk parameter transfer.

## RF Settings

Files:

- `firmware/GCS_TurboFast/GCS_TurboFast.ino`
- `firmware/UAV_TurboFast/UAV_TurboFast.ino`

Key constants:

- `FREQ_MHZ`
- `LORA_BW_KHZ`
- `LORA_CR_DEN`
- `LORA_SYNC`
- `SF_MIN`
- `SF_MAX`
- `TP_MIN`
- `TP_MAX`

Both nodes must match frequency, bandwidth, coding rate, sync word, and supported SF range. TX power must respect the actual RF module PA path and local regulations.

## Arduino Include Layout

Arduino IDE builds each `.ino` from its sketch folder. The sketches therefore include shared headers as:

```cpp
#include "src/common/TelemetryProtoFix.h"
#include "src/common/LinkProfile.h"
#include "src/common/BridgeMavlinkPolicy.h"
```

Do not use absolute includes such as `/common/TelemetryProtoFix.h`.

## MAVLink And Safety Timers

Files:

- `firmware/GCS_TurboFast/GCS_TurboFast.ino`
- `firmware/UAV_TurboFast/UAV_TurboFast.ino`

Important constants:

- `FLIGHT_COMMAND_HOLD_MS`
- `PARAM_SYNC_TIMEOUT_MS`
- `PARAM_SYNC_IDLE_EXIT_MS`
- `BLOCK_FULL_PARAM_SYNC_HIGH_SF`
- `BLOCK_PARAM_READ_HIGH_SF`
- `MP_EXTENDED_SETUP_FEATURES_MAX_SF`
- `PIXHAWK_UART_SILENT_MS`

Do not change these without bench logs showing why the existing value fails.

## Parameter Sync Timing

The following values are policy functions in `LinkProfile.h`, not independent sketch-local constants:

| Function | SF7 | SF8 | SF9 | Purpose |
| --- | --- | --- | --- | --- |
| `linkParamRequestRetryMs()` | 650 ms | 900 ms | 1200 ms | Retry a silent Pixhawk `PARAM_REQUEST_READ` without flooding UART. |
| `linkParamBulkFillWindow()` | 5 | 4 | 3 | Allow bounded prefetch so LoRa TX does not idle during full sync. |
| `linkParamBulkFillGraceMs()` | 80 ms | 130 ms | 190 ms | Release a partially filled bulk packet when no newer parameter value arrives. |
| `linkParamBulkRecords()` | max | up to 8 | up to 6 | Reduce RF airtime per packet at slower SFs. |

Any change to these values must be applied to both GCS and UAV `LinkProfile.h` copies and followed by a header identity check.
