#include <SPI.h>
#include <RadioLib.h>
#include <MAVLink_ardupilotmega.h>
#include <math.h>
#include <string.h>
#include <EEPROM.h>
#include <stddef.h>
#include "src/common/TelemetryProtoFix.h"
#include "src/common/LinkProfile.h"
#include "src/common/BridgeMavlinkPolicy.h"

// =====================================================
// UAV NODE - OPTIMIZED MAVLINK-AWARE LORA TELEMETRY BRIDGE
// Transmits compact telemetry beacons at regular intervals to GCS and forwards GCS command packets
// =====================================================

#define DEFAULT_SF 7   // Spreading Factor (Range: 7 to 12). GCS will automatically scan and lock to this SF. Higher Spreading Factor increases link margin but increases airtime and reduces update rate.
#define DEFAULT_TP 20  // Default UAV TX power in dBm. GCS follows after lock; keep within TP_MIN..TP_MAX.
#define UAV_MASTER_PHY_MODE 1
#define PHY_TEST_FORCE_DEFAULT_SF 1  // 1 = always enforce DEFAULT_SF on boot and write to EEPROM
#define EEPROM_SF_ADDR 0

// Normal-mode telemetry cadence for the controlled SF7-SF9 experiment.
// This value is intentionally shared by SF7, SF8, and SF9 in
// normal telemetry mode so SF/CR comparisons use the same packet
// generation interval. Full parameter sync still uses the separate
// paramSyncTelemetryIntervalForSF() policy below.
#define FIXED_INTERVAL_MS 400UL
#define TELEMETRY_ACK_DECIMATION_ENABLE 1
#define SYNTHETIC_GCS_HEARTBEAT_ENABLE 0  // keep OFF: do not mask ArduPilot FS_GCS failsafe when telemetry link is lost
#define SYNTHETIC_GCS_HEARTBEAT_INTERVAL_MS 1000UL
#define SYNTHETIC_GCS_HEARTBEAT_VALID_MS 10000UL
#define SYNTHETIC_GCS_HEARTBEAT_VALID_MS_SF10 14000UL
#define SYNTHETIC_GCS_HEARTBEAT_VALID_MS_SF11 18000UL
#define SYNTHETIC_GCS_HEARTBEAT_VALID_MS_SF12 25000UL
#define SYNTHETIC_GCS_SYS_ID 255
#define SYNTHETIC_GCS_COMP_ID 190

uint8_t linkMode = LINK_MODE_NORMAL;

// ================= Flight Command Priority Mode =================
#define FLIGHT_COMMAND_HOLD_MS 8000UL
#define COMMAND_DOWNLINK_LISTEN_EVERY_BEACON_SF_MIN 10  // Spreading factor threshold below which downlink listening window is enabled
unsigned long commandModeUntilMs = 0;
uint32_t flightCommandRxCount = 0;
uint32_t flightCommandDownlinkSlotCount = 0;
bool paramSyncActive = false;
unsigned long paramSyncStartMs = 0;
unsigned long paramSyncUntilMs = 0;
uint32_t paramSyncAutoAbortCount = 0;
unsigned long lastParamValueMs = 0;
uint16_t lastParamIndex = 0;
uint16_t lastParamCount = 0;

// ================= Strict Async Parameter Polling Proxy =================
// This proxy intercepts MAVLINK_MSG_ID_PARAM_REQUEST_LIST from Mission Planner.
// It NEVER forwards PARAM_REQUEST_LIST to Pixhawk.
// It requests ArduPilot params via PARAM_REQUEST_READ while keeping exactly one
// outstanding Pixhawk request per index. Unlike the original stop-and-wait
// design, LoRa PARAM_BULK may remain queued while the proxy continues polling as
// long as the bounded RAM window is healthy. This keeps ordering deterministic
// and improves SF7 throughput without allowing queue runaway.

#ifndef MAV_COMP_ID_MISSIONPLANNER
#define MAV_COMP_ID_MISSIONPLANNER 190
#endif

bool     isAsyncParamSyncActive = false;
uint16_t currentAsyncParamIndex = 0;
uint16_t asyncParamTotalCount   = 0;

bool asyncParamWaitingForValue = false;

unsigned long asyncParamLastRequestMs = 0;
unsigned long asyncParamLastValueMs   = 0;

uint8_t asyncParamRequesterSysId  = 255;
uint8_t asyncParamRequesterCompId = MAV_COMP_ID_MISSIONPLANNER;

uint32_t asyncParamPollStartCount   = 0;
uint32_t asyncParamPollDoneCount    = 0;
uint32_t asyncParamPollAbortCount   = 0;
uint32_t asyncParamRequestTxCount   = 0;
uint32_t asyncParamRequestRetryCount = 0;

// Because RadioLib transmit() is blocking in this sketch, this flag is mostly
// a safety guard. It still gives updateAsyncParamPolling() a clean TX-busy check.
volatile bool loraTxInProgress = false;

// ========================================================================
// =================================================================

// ================= Calibration and Configuration Mode Governors =================
#define CAL_CONFIG_HOLD_MS 180000UL
bool calConfigActive = false;
unsigned long calConfigUntilMs = 0;
uint8_t preCalConfigSF = DEFAULT_SF;
uint8_t preCalConfigTP = DEFAULT_TP;
bool calConfigFastConfigRequested = false;
uint32_t calCommandRxCount = 0;
uint32_t magCalProgressRxCount = 0;
uint32_t magCalReportRxCount = 0;
uint32_t calCommandAckRxCount = 0;
uint32_t calStatustextRxCount = 0;

#define PARAM_SYNC_TIMEOUT_MS          600000UL
#define PARAM_SYNC_IDLE_EXIT_MS          15000UL
#define PARAM_SYNC_DRAIN_MAX_MS          1000UL
// Safety policy: full PARAM_REQUEST_LIST is too slow at high SF and can stall telemetry.
// 1 = block full-list parameter sync at SF above FULL_PARAM_SYNC_MAX_SF.
// Full-list and single-read parameter get traffic is blocked at SF10-SF12.
// PARAM_SET remains allowed as a write/command path with PARAM_VALUE acknowledgement.
#define BLOCK_FULL_PARAM_SYNC_HIGH_SF 1
#define FULL_PARAM_SYNC_MAX_SF 9
// Screening: limits setup and log download responses at high spreading factors to prevent link congestion.
// forwarded on SF7-SF9. SF10-SF12 remain command/monitoring links, not bulk/setup links.
#define MP_EXTENDED_SETUP_FEATURES_MAX_SF 9
#define BLOCK_PARAM_READ_HIGH_SF 1
#define PARAM_SYNC_HARD_ABORT_MS_SF11 45000UL
#define PARAM_SYNC_HARD_ABORT_MS_SF12 30000UL
#define PARAM_SYNC_FORCE_SF 7
#define PARAM_SYNC_FORCE_TP TP_MAX
// 0 = Maintain active spreading factor and transmit power during setup/calibration modes.
// 1 = Temporarily override to SF7 and maximum transmit power for maximum responsiveness.
#define FAST_SETUP_FORCE_PHY_ENABLE 1
#define PARAM_SYNC_LOWRAW_INTERVAL_MS 120UL
#define PARAM_SYNC_EXIT_RESTORE_MSADR 1
#define PARAM_SYNC_STREAM_SLOWDOWN_ENABLE 1
#define PARAM_SYNC_STREAM_SLOW_US 1000000L  // 1 Hz: keep GPS/EKF alive if stream slowdown is enabled

uint8_t preParamSyncSF = DEFAULT_SF;
uint8_t preParamSyncTP = DEFAULT_TP;
bool paramSyncFastConfigRequested = false;
bool paramSyncStreamSlowed = false;
uint32_t paramBundleCount = 0;
uint32_t paramBundleBytes = 0;
uint32_t paramBulkQueuedCount = 0;
uint32_t paramBulkTxCount = 0;
uint32_t paramBulkAckCount = 0;
uint32_t paramBulkFailCount = 0;
uint32_t paramBulkQueueDrop = 0;
unsigned long lastParamDebugTxMs = 0;
bool suppressCurrentRawToPixhawk = false;
uint32_t fullParamSyncBlockedHighSfCount = 0;

bool paramWriteAckExpected = false;
char expectedParamId[17] = {0};
unsigned long paramWriteAckUntilMs = 0;
#define PARAM_WRITE_ACK_WINDOW_MS 12000UL
bool paramExtWriteAckExpected = false;
char expectedParamExtId[17] = {0};
unsigned long paramExtWriteAckUntilMs = 0;
#define PARAM_EXT_WRITE_ACK_WINDOW_MS 12000UL

// Protocol constants moved to TelemetryProtoFix.h

// =============================================================================
// LORA PHYSICAL LAYER TUNABLE PARAMETERS (UAV SIDE)
// =============================================================================

// LoRa Hardware SPI Pin Connections
#define LORA_SS    5                  // SPI Slave Select pin
#define LORA_RST   25                 // Radio reset pin
#define LORA_DIO0  26                 // Digital I/O 0 pin (packet RX/TX interrupts)
#define LORA_DIO1  RADIOLIB_NC        // Digital I/O 1 pin (Not Connected)

// LoRa RF Channel Configuration
// [Tuning Guideline] Must match exactly between GCS and UAV nodes.
// Configured here for 420 MHz operation. Must match exactly on GCS and UAV and comply with local spectrum rules.
// Impact: Changing the frequency helps avoid local RF interference or align with legal ISM band standards in your region.
#define FREQ_MHZ      420.0

// LoRa Bandwidth (kHz)
// [Tuning Guideline] Higher bandwidth enables higher data transmission rate (less airtime/latency), 
// but decreases receiver sensitivity and reduces overall range (link budget).
// Options: 125.0, 250.0, 500.0. Recommended default: 500.0 kHz for high throughput.
#define LORA_BW_KHZ   500.0

// LoRa Coding Rate Denominator
// [Tuning Guideline] Code rate is 4/(LORA_CR_DEN). Options: 5 (CR 4/5), 6 (CR 4/6), 7 (CR 4/7), 8 (CR 4/8).
// Impact: Higher CR denominator increases error-correction redundancy (better link robustness in noise), 
// but increases packet airtime and latency.
#define LORA_CR_DEN   5  // CR 4/5 for this diagnostic build. Keep this value identical on GCS and UAV.

// LoRa Sync Word
// [Tuning Guideline] Must match between GCS and UAV. Range: 0x00 to 0xFF.
// Impact: Isolates your network. Only transceivers with the matching sync word can decode each other's packets.
#define LORA_SYNC     0x12

// Spreading Factor Boundaries
// [Tuning Guideline] Spreading Factor range. Min: 7, Max: 12.
// Impact: Higher Spreading Factors increase receiver sensitivity and double the range per step, 
// but exponentially increase packet airtime. At SF12, update rate drops to ~1 Hz.
#define SF_MIN 7
#define SF_MAX 12

// UAV Transmit Power Configurations
// [Tuning Guideline] Range: 10 to 20 dBm. Confirm your SX1278 PA path/regulatory limit before field use.
// Impact: Higher transmit power improves signal strength at the GCS receiver, but increases UAV power consumption.
#define TP_MIN 10
#define TP_MAX 20

// Adaptive Link Rate (MSADR) Transmit Power boundaries
#define MSADR_TP_MIN 10
#define MSADR_TP_MAX 20
#define MSADR_TP_DEFAULT 20

// Adaptive Link Rate threshold adjustments
// [Tuning Guideline] Higher values make the spreading factor adaptation more conservative and stable;
// lower values make it adjust faster/more aggressively to link variations.
#define MSADR_SUCCESS_THRESHOLD 2
#define MSADR_FAIL_THRESHOLD 2

#define TURNAROUND_MS          2UL
#define MIN_SEND_GAP_MS       15UL
#define CONFIG_RETRY_MS      600UL
#define FALLBACK_FAIL_COUNT    8
#define FALLBACK_BEACON_INTERVAL_MS 5000UL
#define CONFIG_STUCK_CANCEL_MS 8000UL
#define RADIO_SOFT_RECOVERY_MS  7000UL
#define RADIO_SOFT_RECOVERY_MIN_GAP_MS 3000UL

// ================= Boot / Autostart Recovery =================
// Auto-recovery: enables the UAV node to recover and re-initialize the LoRa transceiver after power-on.
#define BOOT_STABILIZE_MS                 1500UL
#define LORA_INIT_RETRY_COUNT             8
#define LORA_INIT_RETRY_DELAY_MS          250UL
#define LORA_BOOT_FAIL_RESTART_DELAY_MS   2000UL
#define PIXHAWK_UART_RECOVERY_ENABLE      1
#define PIXHAWK_BOOT_GRACE_MS             30000UL
#define PIXHAWK_UART_SILENT_MS            12000UL
#define PIXHAWK_UART_RECOVERY_MIN_GAP_MS  8000UL

// Packet types moved to TelemetryProtoFix.h

// PROFILE_BEACON moved to TelemetryProtoFix.h

// RAW_MAVLINK_MAX moved to TelemetryProtoFix.h
// PARAM_BULK_MAX_RECORDS moved to TelemetryProtoFix.h
#define PARAM_BULK_QUEUE_SIZE  72
#define PARAM_BULK_TX_INTERVAL_MS 0UL

// During full param sync, telemetry ACK waits were the largest measured stall:
// every 132-byte beacon used to block up to ackTimeoutForSF() while PARAM_BULK
// was already providing frequent GCS->UAV RX opportunities.  Decimate telemetry
// ACKs and use a short telemetry-only RX window; keep PARAM_BULK ACK timeout
// separate for reliability.
#define PARAM_SYNC_TELEM_ACK_EVERY_SF7 0  // 0 = no telemetry ACK wait during full param sync
#define PARAM_SYNC_TELEM_ACK_EVERY_SF8 0
#define PARAM_SYNC_TELEM_ACK_EVERY_SF9 0

// Best-effort parameter-sync debug. It is intentionally rate-limited because
// debug packets compete with PARAM_BULK airtime. Use 32/10000 or disable for
// final release timing tests.
#define PARAM_DEBUG_ENABLE 1
#define PARAM_DEBUG_EVERY_BEACONS 0UL      // Reserved; debug is time-based, not beacon-count based.
#define PARAM_DEBUG_MIN_INTERVAL_MS 20000UL // User-requested: one debug snapshot every ~20 seconds

#define HIGH_QUEUE_SIZE       64
#define LOW_QUEUE_SIZE        160
// LORA_RX_MAX moved to TelemetryProtoFix.h
#define MAG_CAL_COMPASS_MAX   8
#define MAG_CAL_PROGRESS_TX_INTERVAL_MS 70UL
#define STREAM_CONFIG_ENABLE          1
#define STREAM_CONFIG_RETRY_MS     10000UL
#define STREAM_CONFIG_MAX_RETRY        6
#define PARAM_SYNC_LOW_QUEUE_WATERMARK (LOW_QUEUE_SIZE - 2)
#define PIXHAWK_RX_BUFFER_SIZE      8192

#define MOTOR_MONITOR_ENABLE 1
#define SERVO_OUTPUT_MONITOR_INTERVAL_US 1000000L
#define RC_CHANNELS_MONITOR_INTERVAL_US 1000000L
#define ACTUATOR_MONITOR_INTERVAL_US 2000000L
#define ESC_MONITOR_INTERVAL_US 2000000L

#define INTERACTIVE_SETUP_MAIN_STREAM_US   5000000L
#define INTERACTIVE_RC_CHANNELS_INTERVAL_US   1000000L
#define INTERACTIVE_SERVO_OUTPUT_INTERVAL_US  2000000L
#define INTERACTIVE_ACTUATOR_INTERVAL_US      5000000L
#define INTERACTIVE_ESC_INTERVAL_US           5000000L

#define METRICS_SUPPLY_VOLTAGE 3.30f
#define LORA_PREAMBLE_SYMBOLS 12.0f

SPIClass spi(VSPI);
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(LORA_SS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, spiSettings);

#define PIX_RX 16
#define PIX_TX 17
HardwareSerial PixhawkSerial(2);
mavlink_message_t mavMsg;
mavlink_status_t mavStatus;

// =============================================================================
// BRIDGE LINK FAILSAFE PARAMETERS (UAV SIDE)
// =============================================================================

// Enable or disable GCS link failsafe triggering
#define ENABLE_SAFETY_RTL 1

// Flight Controller Failsafe Action Coordinator
// [Tuning Guideline] 0 = FC-first failsafe: ArduPilot's FS_GCS_ENABLE parameter governs the failsafe action;
// ESP32 bridge stays passive. 1 = ESP32 actively injects failsafe flight modes.
#define BRIDGE_FAILSAFE_ENABLE 0

// Safety Failsafe Mode Options
#define BRIDGE_FAILSAFE_ACTION_RTL 1
#define BRIDGE_FAILSAFE_ACTION_LAND 2
#define BRIDGE_FAILSAFE_ACTION_BRAKE 3

// Configured Failsafe Mode
// [Tuning Guideline] Determines what mode ArduPilot is forced into when the telemetry link is lost.
// Options: BRIDGE_FAILSAFE_ACTION_RTL, BRIDGE_FAILSAFE_ACTION_LAND, BRIDGE_FAILSAFE_ACTION_BRAKE.
#define BRIDGE_FAILSAFE_ACTION BRIDGE_FAILSAFE_ACTION_RTL

// Number of consecutive failed packets before initiating failsafe actions
#define LINK_FAIL_RTL_THRESHOLD 5

// Connection stall timeout in milliseconds before GCS link failsafe is triggered
// [Tuning Guideline] Must be configured higher than the beacon interval + maximum packet airtime.
// Recommended range: 5000 ms to 15000 ms. Setting it too low causes transient failsafes under high noise.
#define BRIDGE_FAILSAFE_TIMEOUT_MS 6500UL
#define RTL_REPEAT_INTERVAL_MS 10000UL
#define ARDUPILOT_COPTER_MODE_RTL 6
#define ARDUPILOT_COPTER_MODE_LAND 9
#define ARDUPILOT_COPTER_MODE_BRAKE 17
#define RADIO_SYS_ID 250
#define RADIO_COMP_ID 68

uint8_t consecutiveLinkFails = 0;
unsigned long lastRtlCommandMs = 0;
unsigned long lastFallbackBeaconMs = 0;
unsigned long bootMs = 0;
unsigned long lastPixhawkMavlinkMs = 0;
unsigned long lastPixhawkHeartbeatMs = 0;
unsigned long lastPixhawkUartRecoveryMs = 0;
uint32_t pixhawkUartRecoveryCount = 0;
uint32_t loraInitRetrySuccessCount = 0;

const int SF_LIST[6] = {7, 8, 9, 10, 11, 12};
float P[6] = {0.20f, 0.16f, 0.16f, 0.16f, 0.16f, 0.16f};

int currentSF = SF_MIN;
int previousSF = SF_MIN;
int currentTP = MSADR_TP_DEFAULT;
uint8_t currentProfile = PROFILE_BEACON;

int ackHist[10] = {0};
int histIdx = 0;
int cntACK = 0;
int successStreak = 0;
int failStreak = 0;
int lastAck = 0;

bool configPending = false;
uint8_t pendingSF = SF_MIN;
uint8_t pendingTP = MSADR_TP_DEFAULT;
uint8_t pendingProfile = PROFILE_BEACON;
uint32_t pendingApplyCounter = 0;
unsigned long lastConfigTryMs = 0;
unsigned long configPendingSinceMs = 0;

bool scheduledConfig = false;
uint8_t scheduledSF = SF_MIN;
uint8_t scheduledTP = MSADR_TP_DEFAULT;
uint8_t scheduledProfile = PROFILE_BEACON;
uint32_t scheduledApplyCounter = 0;
unsigned long scheduledConfigSinceMs = 0;

uint32_t counter = 0;
float lastLatency = 0.0f;

unsigned long lastTelemetryTx = 0;
unsigned long lastLowRawTx = 0;
unsigned long lastParamBulkTx = 0;
unsigned long lastAnyTx = 0;
unsigned long lastAckOkMs = 0;
unsigned long lastGcsContactMs = 0;
unsigned long lastSyntheticGcsHeartbeatMs = 0;
unsigned long lastRadioRecoveryMs = 0;
unsigned long lastStreamConfigMs = 0;
uint8_t streamConfigRetry = 0;
bool streamConfigDone = false;

uint32_t highQueueDrop = 0;
uint32_t lowQueueDrop = 0;
uint32_t paramValueDrop = 0;
uint32_t rawCommandDrop = 0;
uint32_t telemetryTxCount = 0;
uint32_t rawTxCount = 0;
uint32_t configTxCount = 0;
uint32_t fallbackBeaconCount = 0;
uint32_t invalidLengthDrop = 0;
uint32_t invalidProtocolDrop = 0;
uint32_t invalidCrcDrop = 0;
uint32_t streamConfigSentCount = 0;
uint32_t streamConfigRetryCount = 0;
uint32_t paramGuardDrop = 0;
uint32_t transparentForwardDrop = 0;
uint32_t oversizedMavlinkDrop = 0;
uint32_t radioSoftRecoveryCount = 0;
uint32_t configStuckCancelCount = 0;
uint32_t securityReplayDrop = 0;

uint16_t securityLastCompactSeqFromGCS = 0;
bool securityHaveCompactSeqFromGCS = false;
unsigned long securityLastCompactSeqMs = 0;

uint32_t telemetryTxAttemptsMetric = 0;
float telemetryTxEnergyMetric_mJ = 0.0f;
float telemetryTxToAMetric_ms = 0.0f;

// Remote GCS-to-UAV link quality measured directly at the UAV node.
// Sent back to the GCS inside the telemetry beacon to populate remote RSSI.
//
// User-requested MP/UI RSSI profile:
// - This only affects the RSSI value exported in the telemetry beacon.
// - It does NOT change LoRa RF power, sensitivity, SF scan, or modem behavior.
// - Keep this enabled only if you intentionally want the GCS/Mission Planner
//   display to see a fixed downlink RSSI around -17 dBm.
#define UAV_FORCE_REMOTE_RSSI_FOR_MP_UI 1
#define UAV_FORCED_REMOTE_RSSI_DBM     (-17.0f)

float lastGcsDownlinkRssi = -125.0f;
float lastGcsDownlinkSnr = -64.0f;
unsigned long lastGcsDownlinkMetricMs = 0;

// VALID_* flag defines moved to TelemetryProtoFix.h (VALID_HEARTBEAT..VALID_LOCAL_VEL, VALID2_*)


// ================= Data Structures for Pixhawk Telemetry Storage =================
struct __attribute__((packed)) PixhawkDataFull {
  // ── Core status ─────────────────────────────────
  uint32_t valid_flags;
  uint8_t  system_id;
  uint8_t  component_id;
  uint8_t  mav_type;
  uint8_t  autopilot;
  uint8_t  base_mode;
  uint32_t custom_mode;
  uint8_t  system_status;

  // ── Attitude ────────────────────────────────────
  float roll;
  float pitch;
  float yaw;
  float rollspeed;
  float pitchspeed;
  float yawspeed;

  // ── Position + NED velocity ──────────────────────
  int32_t  lat;
  int32_t  lon;
  int32_t  alt_mm;
  int32_t  relative_alt_mm;
  int16_t  vx;    // NED North velocity, cm/s
  int16_t  vy;    // NED East  velocity, cm/s
  int16_t  vz;    // NED Down  velocity, cm/s
  uint16_t hdg;   // Compass heading, cdeg

  // ── Airdata ─────────────────────────────────────
  float    airspeed;
  float    groundspeed;
  int16_t  heading;   // Heading from VFR_HUD (deg)
  uint16_t throttle;  // 0-100%
  float    climb;

  // ── Battery ─────────────────────────────────────
  uint16_t voltage_battery;   // mV
  int16_t  current_battery;   // cA
  int8_t   battery_remaining; // %

  // ── EKF + GPS ───────────────────────────────────
  uint16_t    ekf_flags;
  GpsRawDataFull gps;

  // ── Vibration [NEW] ─────────────────────────────
  uint32_t valid_flags2;       // Extended validity bits (mirrors VALID2_* defines)
  float    vibe_x;             // Vibration X, m/s²
  float    vibe_y;             // Vibration Y, m/s²
  float    vibe_z;             // Vibration Z, m/s²
  uint32_t accel_clip_0;       // Accel clipper count axis 0
  uint32_t accel_clip_1;       // Accel clipper count axis 1
  uint32_t accel_clip_2;       // Accel clipper count axis 2

  // ── RC Channels [NEW] ───────────────────────────
  uint16_t rc_channels[16];   // Raw PWM µs, channels 1-16
};


#define RAW_PKT_HEADER_LEN (sizeof(PacketHeader) + 1)
#define PARAM_BULK_BASE_LEN (sizeof(PacketHeader) + 4)
#define PARAM_BULK_LEN(n) (PARAM_BULK_BASE_LEN + ((uint16_t)(n) * sizeof(CompactParamValue)))

#include "UAVRadioHelpers.h"
#include "UAVParamQueue.h"
#include "UAVCalibHelpers.h"
#include "BeaconFillHelpers.h"

uint8_t profileForSF(int sf);
bool isFlightActionCommandId(uint16_t command);
bool peekHighRawPacket(MavlinkRawPacket &pkt);
bool peekLowRawPacket(MavlinkRawPacket &pkt);
void pushExistingHighRawPacket(const MavlinkRawPacket &pkt);
bool rawPacketContainsCommandAck(const MavlinkRawPacket &pkt, uint16_t command);


PixhawkDataFull pixDataFull;

// -----------------------------------------------------------------------------
// Autopilot identity lock
// -----------------------------------------------------------------------------
// Do not infer PARAM_SET / COMMAND_LONG targets from arbitrary MAVLink frames.
// Some ArduPilot setups emit messages from non-autopilot components; if those
// overwrite the command target, Mission Planner actions such as PARAM_SET or
// Motor Test can be sent to the wrong component and time out.  The bridge locks
// the command target only from HEARTBEAT frames that identify an autopilot.
uint8_t autopilotTargetSysId = 1;
uint8_t autopilotTargetCompId = MAV_COMP_ID_AUTOPILOT1;
bool autopilotIdentityLocked = false;
uint32_t autopilotIdentityLockCount = 0;

// Rate limit raw diagnostic MAVLink passthrough during full parameter sync.
// The compact beacon carries lightweight EKF/VIBRATION summaries, but Mission
// Planner's diagnostic windows need the raw messages periodically.  Forwarding
// them at 1 Hz preserves observability without competing heavily with PARAM_BULK.
#define DIAG_RAW_SYNC_INTERVAL_MS   1000UL
#define DIAG_RAW_NORMAL_INTERVAL_MS  500UL
unsigned long lastRawEkfForwardMs = 0;
unsigned long lastRawVibrationForwardMs = 0;

MavlinkRawPacket highQueue[HIGH_QUEUE_SIZE];
uint16_t highHead = 0, highTail = 0, highCount = 0;
MavlinkRawPacket lowQueue[LOW_QUEUE_SIZE];
uint16_t lowHead = 0, lowTail = 0, lowCount = 0;
ParamBulkPacket paramBulkQueue[PARAM_BULK_QUEUE_SIZE];
uint16_t paramBulkHead = 0, paramBulkTail = 0, paramBulkCount = 0;

#ifdef MAVLINK_MSG_ID_MAG_CAL_PROGRESS
mavlink_message_t magCalProgressCache[MAG_CAL_COMPASS_MAX];
bool magCalProgressPending[MAG_CAL_COMPASS_MAX] = {false};
uint8_t magCalProgressNextId = 0;
unsigned long lastMagCalProgressTxMs = 0;
uint32_t magCalProgressCachedCount = 0;
uint32_t magCalProgressBundleTxCount = 0;
uint32_t magCalProgressBundleAckCount = 0;
uint32_t magCalProgressBundleFailCount = 0;
#endif

// ================= Utility / CRC =================
// Local helper wrappers delegating to TelemetryProtoFix.h

static inline uint8_t packRadioParams78(uint8_t sf, uint8_t tp, uint8_t profile) {
  return packRadioParams78(sf, tp, profile, SF_MIN, SF_MAX, TP_MIN, TP_MAX);
}

static inline uint8_t unpackSf78(uint8_t packed) {
  return unpackSf78(packed, SF_MIN, SF_MAX);
}

static inline uint8_t unpackTp78(uint8_t packed) {
  return unpackTp78(packed, TP_MIN, TP_MAX);
}

bool validatePacket(const uint8_t *buf, size_t len) {
  return validatePacketInternal(buf, len, invalidLengthDrop, invalidProtocolDrop, invalidCrcDrop);
}

bool acceptRollingCompactSeqFromGCS(uint16_t seq) {
#if SECURITY_ANTI_REPLAY_ENABLE
  if (!securityHaveCompactSeqFromGCS) {
    securityHaveCompactSeqFromGCS = true;
    securityLastCompactSeqFromGCS = seq;
    securityLastCompactSeqMs = millis();
    return true;
  }

  int16_t delta = (int16_t)(seq - securityLastCompactSeqFromGCS);
  if (delta > 0) {
    securityLastCompactSeqFromGCS = seq;
    securityLastCompactSeqMs = millis();
    return true;
  }

  // Resets sequence counter if GCS restarts after a prolonged link idle state.
  unsigned long idleMs = millis() - securityLastCompactSeqMs;
  if (idleMs > SECURITY_REBOOT_GRACE_MS && seq < 16) {
    securityLastCompactSeqFromGCS = seq;
    securityLastCompactSeqMs = millis();
    return true;
  }

  securityReplayDrop++;
  return false;
#else
  (void)seq;
  return true;
#endif
}

void copyParamId(char *dest, const char *src) { memcpy(dest, src, 16); dest[16] = 0; }
void copyParamId16(char *dest, const char *src) { memset(dest, 0, 16); strncpy(dest, src, 16); }
bool sameParamId(const char *a, const char *b) { return strncmp(a, b, 16) == 0; }

bool isInteractiveSetupParamId(const char *id) {
  return bridgeIsInteractiveSetupParamId(id);
}

void flushLowQueue() { lowHead = lowTail = lowCount = 0; }
void flushParamBulkQueue() { paramBulkHead = paramBulkTail = paramBulkCount = 0; }

bool radioIsCurrentlyTransmitting() {
  return loraTxInProgress;
}

int16_t radioTransmitBlocking(const uint8_t *buf, size_t len) {
  loraTxInProgress = true;
  int16_t state = radio.transmit((uint8_t *)buf, len);
  loraTxInProgress = false;
  return state;
}

void resetAsyncParamSyncProxy(bool countAbort) {
  if (countAbort && isAsyncParamSyncActive) {
    asyncParamPollAbortCount++;
  }

  isAsyncParamSyncActive = false;
  currentAsyncParamIndex = 0;
  asyncParamTotalCount = 0;

  asyncParamWaitingForValue = false;
  asyncParamLastRequestMs = 0;
  asyncParamLastValueMs = 0;

  asyncParamRequesterSysId = 255;
  asyncParamRequesterCompId = MAV_COMP_ID_MISSIONPLANNER;
}

void initPixhawkSerialPort() {
  PixhawkSerial.end();
  delay(30);
  PixhawkSerial.setRxBufferSize(PIXHAWK_RX_BUFFER_SIZE);
  PixhawkSerial.begin(115200, SERIAL_8N1, PIX_RX, PIX_TX);
  while (PixhawkSerial.available()) PixhawkSerial.read();
}

void configurePixhawkMessageIntervalsNormal();  // forward declaration for autostart UART recovery
void recoverPixhawkSerialIfSilent() {
#if PIXHAWK_UART_RECOVERY_ENABLE
  unsigned long now = millis();
  if (bootMs == 0 || now - bootMs < PIXHAWK_BOOT_GRACE_MS) return;
  if (lastPixhawkMavlinkMs != 0 && now - lastPixhawkMavlinkMs < PIXHAWK_UART_SILENT_MS) return;
  if (now - lastPixhawkUartRecoveryMs < PIXHAWK_UART_RECOVERY_MIN_GAP_MS) return;

  lastPixhawkUartRecoveryMs = now;
  pixhawkUartRecoveryCount++;
  Serial.print("[RECOVERY UAV] Pixhawk UART silent, reinit count=");
  Serial.println(pixhawkUartRecoveryCount);

  initPixhawkSerialPort();
  memset(&mavStatus, 0, sizeof(mavStatus));
  memset(&mavMsg, 0, sizeof(mavMsg));
  configurePixhawkMessageIntervalsNormal();
  lastStreamConfigMs = millis();
  streamConfigRetry = 1;
  streamConfigRetryCount++;
#endif
}

void proposeConfig(uint8_t nextSF, uint8_t nextTP);
void applyConfig(uint8_t sf, uint8_t tp, uint8_t profile);
void scheduleConfigApply(uint8_t sf, uint8_t tp, uint8_t profile, uint32_t applyCounter);
void applyScheduledConfigAfterAckedTelemetry(uint32_t ackedCounter);
bool sendFallbackBeaconIfNeeded();
void configurePixhawkMessageIntervalsNormal();
void configurePixhawkMessageIntervalsParamSync();
void configurePixhawkMessageIntervalsInteractiveSetup();
void resetAsyncParamSyncProxy(bool countAbort);
void startAsyncParamSyncProxy(uint8_t requesterSysId, uint8_t requesterCompId);
void sendAsyncParamRequestReadToPixhawk(uint16_t paramIndex);
unsigned long asyncParamRequestRetryForSF(int sf);
uint8_t paramBulkFillWindowForSF(int sf);

bool radioIsCurrentlyTransmitting();
int16_t radioTransmitBlocking(const uint8_t *buf, size_t len);

uint8_t getTargetSysId();
uint8_t getTargetCompId();
void writeMavlinkToPixhawk(const mavlink_message_t &msg);
void enterParamSyncStreamMode();
void exitParamSyncStreamMode();
void resetLinkQualityAfterParamSync();
bool calibrationConfigModeActive();
bool activeConfigOrParamMode();
void startCalibrationConfigMode();
void stopCalibrationConfigMode();
void updateCalibrationConfigMode();
bool sendTelemetryPacketToGCS();

bool isParamSyncOperational() { return paramSyncActive; }
bool calibrationConfigModeActive() { return calConfigActive && millis() < calConfigUntilMs; }
bool activeConfigOrParamMode() { return paramSyncActive || calibrationConfigModeActive(); }

void resetLinkQualityAfterParamSync() {
  for (uint8_t i = 0; i < 10; i++) ackHist[i] = 1;
  histIdx = 0; cntACK = 10; successStreak = 0; failStreak = 0; consecutiveLinkFails = 0;
}

void startCalibrationConfigMode() {
  bool wasActive = calibrationConfigModeActive();
  if (!wasActive) { preCalConfigSF = currentSF; preCalConfigTP = currentTP; calConfigFastConfigRequested = false; }
  calConfigActive = true;
  calConfigUntilMs = millis() + CAL_CONFIG_HOLD_MS;
  linkMode = LINK_MODE_CALIBRATION;
  if (!wasActive) {
    configurePixhawkMessageIntervalsInteractiveSetup();
    paramSyncStreamSlowed = true;
    streamConfigDone = true;
    if (FAST_SETUP_FORCE_PHY_ENABLE && !calConfigFastConfigRequested) { proposeConfig(PARAM_SYNC_FORCE_SF, PARAM_SYNC_FORCE_TP); calConfigFastConfigRequested = true; }
  }
}

void stopCalibrationConfigMode() {
  calConfigActive = false; calConfigUntilMs = 0; calConfigFastConfigRequested = false;
  if (paramSyncActive) { configurePixhawkMessageIntervalsParamSync(); linkMode = LINK_MODE_PARAM_SYNC; return; }
  configurePixhawkMessageIntervalsNormal();
  paramSyncStreamSlowed = false; streamConfigDone = false; streamConfigRetry = 0; lastStreamConfigMs = 0;
  linkMode = LINK_MODE_NORMAL;
  resetLinkQualityAfterParamSync();
  if (FAST_SETUP_FORCE_PHY_ENABLE && currentSF == PARAM_SYNC_FORCE_SF && currentTP == PARAM_SYNC_FORCE_TP) proposeConfig(preCalConfigSF, preCalConfigTP);
}

void updateCalibrationConfigMode() { if (calConfigActive && millis() > calConfigUntilMs) stopCalibrationConfigMode(); }

unsigned long paramSyncNoValueExitForSF(uint8_t sf) {
  return linkParamNoValueTimeoutMs(sf);
}

unsigned long paramSyncIdleExitForSF(uint8_t sf) {
  return linkParamIdleTimeoutMs(sf);
}

unsigned long paramSyncLinkStallForSF(uint8_t sf) {
  return linkParamStallTimeoutMs(sf);
}

unsigned long paramSyncHardAbortForSF(uint8_t sf) {
  if (sf >= 12) return PARAM_SYNC_HARD_ABORT_MS_SF12;
  if (sf == 11) return PARAM_SYNC_HARD_ABORT_MS_SF11;
  return 0UL;
}

bool enqueueMavlinkHighForGCS(const mavlink_message_t &msg);
void purgeQueuedCommandAckForGCS(uint16_t command);

bool fullParamSyncAllowedAtCurrentSF() {
#if BLOCK_FULL_PARAM_SYNC_HIGH_SF
  return linkSfAllowsParamGet((uint8_t)currentSF);
#else
  return true;
#endif
}

bool highSfParamTrafficBlocked() {
#if BLOCK_PARAM_READ_HIGH_SF
  return !linkSfAllowsParamGet((uint8_t)currentSF);
#else
  return false;
#endif
}

void enqueueBridgeStatusText(uint8_t severity, const char *text) {
  mavlink_message_t stMsg;
  mavlink_statustext_t st = {};
  st.severity = severity;
  strncpy(st.text, text, sizeof(st.text) - 1);
  st.text[sizeof(st.text) - 1] = '\0';
  mavlink_msg_statustext_encode(RADIO_SYS_ID, RADIO_COMP_ID, &stMsg, &st);
  enqueueMavlinkHighForGCS(stMsg);
}

void flushHighQueueForRecovery() { highHead = highTail = highCount = 0; }

void abortParamSyncRecovery(const char *reason) {
  (void)reason;
  paramSyncAutoAbortCount++;
  configPending = false; scheduledConfig = false; configPendingSinceMs = 0; scheduledConfigSinceMs = 0; lastConfigTryMs = 0;
  flushLowQueue();
  flushParamBulkQueue();
  if (highCount > (HIGH_QUEUE_SIZE / 2)) flushHighQueueForRecovery();
  exitParamSyncStreamMode();
  paramSyncActive = false;
  paramSyncStartMs = 0;
  lastParamValueMs = 0; lastParamIndex = 0; lastParamCount = 0; paramSyncFastConfigRequested = false;
  // Also cancel the async polling proxy so it does not fire after the sync window ends.
  resetAsyncParamSyncProxy(true);
  resetLinkQualityAfterParamSync();
  if (calibrationConfigModeActive()) linkMode = LINK_MODE_CALIBRATION;
  else linkMode = LINK_MODE_NORMAL;
  lastTelemetryTx = 0; lastLowRawTx = millis(); lastParamBulkTx = millis(); lastAnyTx = 0;
  radio.standby();
  delay(5);
}

void startParamSync() {
  bool wasActive = paramSyncActive;
  if (!wasActive) { preParamSyncSF = currentSF; preParamSyncTP = currentTP; paramSyncFastConfigRequested = false; paramSyncStartMs = millis(); lastParamValueMs = 0; lastParamIndex = 0; lastParamCount = 0; enterParamSyncStreamMode(); }
  paramSyncActive = true;
  linkMode = LINK_MODE_PARAM_SYNC;
  paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
  if (FAST_SETUP_FORCE_PHY_ENABLE && !paramSyncFastConfigRequested) { proposeConfig(PARAM_SYNC_FORCE_SF, PARAM_SYNC_FORCE_TP); paramSyncFastConfigRequested = true; }
}

void stopParamSync() {
  configPending = false; scheduledConfig = false; configPendingSinceMs = 0; scheduledConfigSinceMs = 0; lastConfigTryMs = 0;
  flushLowQueue(); flushParamBulkQueue(); exitParamSyncStreamMode();
  paramSyncActive = false; paramSyncStartMs = 0; lastParamValueMs = 0; lastParamIndex = 0; lastParamCount = 0; paramSyncFastConfigRequested = false;
  // Ensure the async proxy does not fire after the sync window closes.
  resetAsyncParamSyncProxy(false);
  resetLinkQualityAfterParamSync();
  if (calibrationConfigModeActive()) { linkMode = LINK_MODE_CALIBRATION; return; }
  linkMode = LINK_MODE_NORMAL;
  uint8_t targetSF = preParamSyncSF, targetTP = preParamSyncTP;
  if (targetSF < SF_MIN) targetSF = SF_MIN; if (targetSF > SF_MAX) targetSF = SF_MAX;
  if (targetTP < TP_MIN) targetTP = TP_MIN; if (targetTP > TP_MAX) targetTP = TP_MAX;
  if (targetSF != currentSF || targetTP != currentTP) proposeConfig(targetSF, targetTP);
  lastTelemetryTx = 0; lastAnyTx = 0;
  for (int retry = 0; retry < 3; retry++) { radio.standby(); delay(10); if (sendTelemetryPacketToGCS()) break; delay(50); }
}

void updateParamSyncTimeout() {
  if (!paramSyncActive) return;
  unsigned long now = millis();

  if (now > paramSyncUntilMs) {
    abortParamSyncRecovery("param_timeout");
    return;
  }

  unsigned long hardAbortMs = paramSyncHardAbortForSF(currentSF);
  if (hardAbortMs > 0 && paramSyncStartMs > 0 && now - paramSyncStartMs > hardAbortMs) {
    abortParamSyncRecovery("param_hard_abort_high_sf");
    return;
  }

  // Async proxy note:
  // For PARAM_REQUEST_LIST we intentionally suppress Pixhawk's full dump and poll one index at a time.
  // Therefore "no first PARAM_VALUE yet" is only a fault after a generous SF-dependent window.
  if (paramSyncStartMs > 0 && lastParamValueMs == 0 && now - paramSyncStartMs > paramSyncNoValueExitForSF(currentSF)) {
    abortParamSyncRecovery("no_param_value");
    return;
  }

  // If the GCS side stops acknowledging packets, exit param mode so normal telemetry can recover.
  if (lastAckOkMs > 0 && now - lastAckOkMs > paramSyncLinkStallForSF(currentSF)) {
    abortParamSyncRecovery("link_stall");
    return;
  }

  // End condition for both legacy individual reads and async proxy.
  // lastParamIndex/lastParamCount are updated only when a PARAM_VALUE is accepted for LoRa delivery.
  if (lastParamCount > 0 && lastParamIndex >= (lastParamCount - 1)) {
    if (lowCount == 0 && paramBulkCount == 0) {
      stopParamSync();
      return;
    }
    if (lastParamValueMs > 0 && now - lastParamValueMs > PARAM_SYNC_DRAIN_MAX_MS) {
      stopParamSync();
      return;
    }
    return;
  }

  // Legacy/interactive param reads can still idle out. Do NOT abort while the async proxy is active,
  // because it may be waiting for LoRa queue drain before polling the next index.
  if (lastParamValueMs > 0 && !isAsyncParamSyncActive &&
      now - lastParamValueMs > paramSyncIdleExitForSF(currentSF)) {
    abortParamSyncRecovery("param_idle_fast_restore");
    return;
  }
}

// =============================================================================
// Strict Async Parameter Polling Proxy
// =============================================================================
void startAsyncParamSyncProxy(uint8_t requesterSysId, uint8_t requesterCompId) {
  uint8_t reqSys  = requesterSysId  ? requesterSysId  : 255;
  uint8_t reqComp = requesterCompId ? requesterCompId : MAV_COMP_ID_MISSIONPLANNER;

  /*
   * H1 guard / idempotent transaction start:
   * Mission Planner can retry PARAM_REQUEST_LIST while a full sync is already
   * running. Restarting here would flush PARAM_BULK, reset index to zero, and
   * make "Getting Params" appear stuck/restarting.
   *
   * Treat any PARAM_REQUEST_LIST received while paramSyncActive as a keepalive:
   * update requester identity only, but never reset queues/cursor.
   */
  if (paramSyncActive) {
    asyncParamRequesterSysId  = reqSys;
    asyncParamRequesterCompId = reqComp;
    return;
  }

  // Defensive cleanup for impossible/stale states. stopParamSync() and abort
  // normally reset the async proxy, but this protects future refactors.
  if (isAsyncParamSyncActive) {
    resetAsyncParamSyncProxy(false);
  }

  // New transaction: clear stale downlink payloads once, then start from index 0.
  flushLowQueue();
  flushParamBulkQueue();

  asyncParamRequesterSysId  = reqSys;
  asyncParamRequesterCompId = reqComp;

  isAsyncParamSyncActive = true;
  currentAsyncParamIndex = 0;
  asyncParamTotalCount = 0;
  asyncParamWaitingForValue = false;
  asyncParamLastRequestMs = 0;
  asyncParamLastValueMs = millis();
  asyncParamPollStartCount++;

  startParamSync();
}

void sendAsyncParamRequestReadToPixhawk(uint16_t paramIndex) {
  mavlink_message_t req;
  mavlink_param_request_read_t pr = {};

  pr.target_system    = getTargetSysId();
  pr.target_component = getTargetCompId();
  pr.param_index      = (int16_t)paramIndex;
  memset(pr.param_id, 0, sizeof(pr.param_id));

  // Use the Mission Planner sysid/compid as sender so ArduPilot treats this as a normal GCS request.
  mavlink_msg_param_request_read_encode(
    asyncParamRequesterSysId,
    asyncParamRequesterCompId,
    &req,
    &pr
  );

  writeMavlinkToPixhawk(req);
  asyncParamRequestTxCount++;
}

void updateAsyncParamPolling() {
  if (!isAsyncParamSyncActive) return;
  if (!paramSyncActive) return;
  if (configPending || scheduledConfig) return;

  // Stop requesting new indexes only after every known parameter has been
  // accepted into the outbound PARAM_BULK path. Draining LoRa is handled by
  // updateParamSyncTimeout()/stopParamSync(), not by this poller.
  if (asyncParamTotalCount > 0 && currentAsyncParamIndex >= asyncParamTotalCount) {
    isAsyncParamSyncActive = false;
    asyncParamWaitingForValue = false;
    asyncParamPollDoneCount++;
    return;
  }

  /*
   * Balanced anti-race backpressure:
   * 1) Do not let ordinary low-priority MAVLink packets block param polling.
   * 2) Do not poll so aggressively that PARAM_BULK RAM becomes a retransmit
   *    backlog. The queue is the handoff boundary between Pixhawk UART and LoRa.
   * 3) Keep the newest bulk open until it is full, or until the SF-specific
   *    fill grace expires in loop(); this preserves 7-9 records/bulk at SF7.
   */
  if (lowCount >= PARAM_SYNC_LOW_QUEUE_WATERMARK) return;
  if (radioIsCurrentlyTransmitting()) return;

  if (paramBulkCount != 0) {
    uint8_t limit = paramBulkRecordsLimitForCurrentSF();
    if (limit == 0) return;

    uint16_t newestIndex = (paramBulkHead + PARAM_BULK_QUEUE_SIZE - 1) % PARAM_BULK_QUEUE_SIZE;
    ParamBulkPacket &newest = paramBulkQueue[newestIndex];

    bool newestBulkFull = (newest.count >= limit);
    bool fillWindowFull = (paramBulkCount >= paramBulkFillWindowForSF(currentSF));
    bool queueAlmostFull = (paramBulkCount >= (PARAM_BULK_QUEUE_SIZE - 2));

    // Only pause Pixhawk polling when the LoRa-side holding area is actually
    // saturated. This avoids the old stop-and-wait behavior while preventing
    // memory pressure / out-of-order races.
    if ((newestBulkFull && fillWindowFull) || queueAlmostFull) return;
  }

  unsigned long now = millis();

  // One outstanding Pixhawk request per index. Retry only when Pixhawk is
  // silent. This prevents UART floods and keeps PARAM_VALUE ordering stable.
  if (asyncParamWaitingForValue) {
    if (asyncParamLastRequestMs > 0 && now - asyncParamLastRequestMs < asyncParamRequestRetryForSF(currentSF)) {
      return;
    }
    asyncParamRequestRetryCount++;
  }

  sendAsyncParamRequestReadToPixhawk(currentAsyncParamIndex);
  asyncParamWaitingForValue = true;
  asyncParamLastRequestMs = now;
}

void updateParamWriteAckState() {
  unsigned long now = millis();
  if (paramWriteAckExpected && now > paramWriteAckUntilMs) { paramWriteAckExpected = false; expectedParamId[0] = 0; }
  if (paramExtWriteAckExpected && now > paramExtWriteAckUntilMs) { paramExtWriteAckExpected = false; expectedParamExtId[0] = 0; }
}

bool canAcceptParamValueToLowQueue() {
  if (!paramSyncActive) return true;
  if (lowCount >= PARAM_SYNC_LOW_QUEUE_WATERMARK && paramBulkCount >= (PARAM_BULK_QUEUE_SIZE - 2)) { paramGuardDrop++; paramValueDrop++; return false; }
  return true;
}

uint8_t profileForSF(int sf) { (void) sf; return PROFILE_BEACON; }

// ================= Parameter Synchronization Acceleration Helpers =================
unsigned long normalTelemetryIntervalForSF(uint8_t sf) {
  return linkNormalTelemetryIntervalMs(sf);
}

unsigned long paramSyncTelemetryIntervalForSF(uint8_t sf) {
  return linkParamSyncTelemetryIntervalMs(sf);
}

unsigned long intervalForSF(int sf) {
  uint8_t s = (sf < SF_MIN) ? SF_MIN : ((sf > SF_MAX) ? SF_MAX : (uint8_t)sf);
  return paramSyncActive ? paramSyncTelemetryIntervalForSF(s) : normalTelemetryIntervalForSF(s);
}

unsigned long paramBulkFillGraceForSF(int sf) {
  return linkParamBulkFillGraceMs((uint8_t)sf);
}

unsigned long asyncParamRequestRetryForSF(int sf) {
  return linkParamRequestRetryMs((uint8_t)sf);
}

uint8_t paramBulkFillWindowForSF(int sf) {
  return linkParamBulkFillWindow((uint8_t)sf);
}

unsigned long lowRawIntervalForSF(int sf) {
  (void) sf;
  if (paramSyncActive) return PARAM_SYNC_LOWRAW_INTERVAL_MS;
  if (calibrationConfigModeActive()) return 50UL;
  return 500UL;
}

unsigned long ackTimeoutForSF(uint8_t sf) {
  return linkRawAckTimeoutMs(sf);
}

unsigned long paramBulkAckTimeoutForSF(uint8_t sf) {
  return linkParamBulkAckTimeoutMs(sf);
}

unsigned long commandDownlinkTimeoutForSF(uint8_t sf) {
  // V14: Downlink slot is no longer kept open for a long duration for each high-SF beacon.
  // This window is only used when command priority is active; SF12 still uses the scheduled ACK-slot.
  if (sf >= 12) return 480UL;
  if (sf == 11) return 620UL;
  if (sf == 10) return 420UL;
  return 220UL;
}

bool commandModeActive() { return commandModeUntilMs != 0 && millis() < commandModeUntilMs; }

bool shouldListenForCommandDownlink(uint8_t sf) {
  (void)sf;
  // V14 delay/force fix:
  // GCS hotfix only transmits downlink command on scheduled ACK slots. Therefore, opportunistic
  // RX windows outside ACK slots only waste 420-620 ms on SF10-SF11 and make Mission Planner UI lag.
  // All SFs now listen for GCS downlink through expectAck/ACK-slot timing only.
  return false;
}

uint8_t telemetryAckEveryForSF(uint8_t sf) {
#if TELEMETRY_ACK_DECIMATION_ENABLE
  /*
   * Telemetry ACK policy:
   *
   * Normal mode keeps ACK every beacon to preserve fast downlink command slots.
   * During full parameter sync, PARAM_BULK packets already open frequent RX
   * windows for GCS ACK/interactive commands. Waiting for a telemetry ACK on
   * every beacon was measured to stall the main loop for hundreds of ms and
   * reduced SF7 sync speed. Therefore telemetry ACKs are decimated only while
   * paramSyncActive is true.
   */
  if (paramSyncActive) {
    if (sf <= 7) return PARAM_SYNC_TELEM_ACK_EVERY_SF7;
    if (sf == 8) return PARAM_SYNC_TELEM_ACK_EVERY_SF8;
    if (sf == 9) return PARAM_SYNC_TELEM_ACK_EVERY_SF9;
  }
  if (sf >= 12) return 2;
  if (sf == 11) return 1;
  if (sf == 10) return 1;
#endif
  return 1;
}

bool shouldExpectTelemetryAck(uint32_t counterValue, uint8_t sf) {
  (void)counterValue;
  if (paramSyncActive) return false;  // PARAM_BULK ACK is the reliability path during full sync.
  uint8_t every = telemetryAckEveryForSF(sf);
  if (every == 0) return false;
  if (every <= 1) return true;
  return (counterValue % every) == 0;
}

unsigned long telemetryAckTimeoutForSF(uint8_t sf) {
  /*
   * Telemetry ACK timeout is intentionally shorter than PARAM_BULK ACK timeout.
   * A missed telemetry ACK is not fatal during param sync; spending 650+ ms
   * waiting for it was the dominant reason SF7 sync exceeded 3 minutes.
   */
  if (paramSyncActive) {
    if (sf <= 9) return linkTelemetryAckTimeoutMs(sf);
  }
  return ackTimeoutForSF(sf);
}

uint8_t selectTelemetryProfile() { return PROFILE_BEACON; }

TelemetryMeta makeTelemetryMeta() {
  TelemetryMeta meta = {};
  meta.cnt_ack = cntACK;
  meta.success_streak = successStreak;
  meta.fail_streak = failStreak;
  meta.telem_tx_energy_mJ_x100 = (uint32_t)(telemetryTxEnergyMetric_mJ * 100.0f + 0.5f);
  return meta;
}
// =====================================================

// ================= Queue (sederhana) =================
bool enqueueHighRawPacket(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > RAW_MAVLINK_MAX) { rawCommandDrop++; return false; }
  if (highCount >= HIGH_QUEUE_SIZE) {
    flushLowQueue();
  }
  if (highCount >= HIGH_QUEUE_SIZE) {
    highQueueDrop++;
    rawCommandDrop++;
    return false;
  }
  MavlinkRawPacket &pkt = highQueue[highHead];
  memset(&pkt, 0, sizeof(pkt));
  initHeader(pkt.hdr, PKT_MAVLINK_RAW);
  pkt.len = len; memcpy(pkt.payload, data, len);
  finalizePacketCrc(&pkt, RAW_PKT_HEADER_LEN + pkt.len);
  highHead = (highHead + 1) % HIGH_QUEUE_SIZE; highCount++;
  return true;
}
bool enqueueLowRawPacket(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > RAW_MAVLINK_MAX) { lowQueueDrop++; return false; }
  if (lowCount >= LOW_QUEUE_SIZE) { lowTail = (lowTail + 1) % LOW_QUEUE_SIZE; lowCount--; lowQueueDrop++; }
  MavlinkRawPacket &pkt = lowQueue[lowHead];
  memset(&pkt, 0, sizeof(pkt));
  initHeader(pkt.hdr, PKT_MAVLINK_RAW);
  pkt.len = len; memcpy(pkt.payload, data, len);
  finalizePacketCrc(&pkt, RAW_PKT_HEADER_LEN + pkt.len);
  lowHead = (lowHead + 1) % LOW_QUEUE_SIZE; lowCount++;
  return true;
}
bool enqueueLowRawPacketBundled(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > RAW_MAVLINK_MAX) { lowQueueDrop++; return false; }
  if (paramSyncActive && lowCount > 0) {
    uint16_t lastIndex = (lowHead + LOW_QUEUE_SIZE - 1) % LOW_QUEUE_SIZE;
    MavlinkRawPacket &lastPkt = lowQueue[lastIndex];
    if (lastPkt.hdr.type == PKT_MAVLINK_RAW && (uint16_t)lastPkt.len + len <= RAW_MAVLINK_MAX) {
      memcpy(&lastPkt.payload[lastPkt.len], data, len);
      lastPkt.len += len;
      finalizePacketCrc(&lastPkt, RAW_PKT_HEADER_LEN + lastPkt.len);
      paramBundleCount++; paramBundleBytes += len;
      return true;
    }
  }
  return enqueueLowRawPacket(data, len);
}

// Bundles multiple high-priority flight controller MAVLink responses into single packets:
// messages into one LoRa raw packet when possible. This reduces airtime and queue
// depth without inventing/synthesizing vehicle state.
bool enqueueHighRawPacketBundled(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > RAW_MAVLINK_MAX) { rawCommandDrop++; return false; }
  if (highCount > 0) {
    uint16_t lastIndex = (highHead + HIGH_QUEUE_SIZE - 1) % HIGH_QUEUE_SIZE;
    MavlinkRawPacket &lastPkt = highQueue[lastIndex];
    if (lastPkt.hdr.type == PKT_MAVLINK_RAW && (uint16_t)lastPkt.len + len <= RAW_MAVLINK_MAX) {
      memcpy(&lastPkt.payload[lastPkt.len], data, len);
      lastPkt.len += len;
      finalizePacketCrc(&lastPkt, RAW_PKT_HEADER_LEN + lastPkt.len);
      return true;
    }
  }
  return enqueueHighRawPacket(data, len);
}
bool peekHighRawPacket(MavlinkRawPacket &pkt) { if (highCount == 0) return false; pkt = highQueue[highTail]; return true; }
bool peekLowRawPacket(MavlinkRawPacket &pkt) { if (lowCount == 0) return false; pkt = lowQueue[lowTail]; return true; }
void popHighRawPacket() { if (highCount == 0) return; highTail = (highTail + 1) % HIGH_QUEUE_SIZE; highCount--; }
void popLowRawPacket() { if (lowCount == 0) return; lowTail = (lowTail + 1) % LOW_QUEUE_SIZE; lowCount--; }
void pushExistingHighRawPacket(const MavlinkRawPacket &pkt) {
  if (highCount >= HIGH_QUEUE_SIZE) return;
  highQueue[highHead] = pkt;
  highHead = (highHead + 1) % HIGH_QUEUE_SIZE;
  highCount++;
}

bool rawPacketContainsCommandAck(const MavlinkRawPacket &pkt, uint16_t command) {
  if (pkt.len == 0 || pkt.len > RAW_MAVLINK_MAX) return false;
  mavlink_message_t parsedMsg;
  mavlink_status_t parsedStatus;
  memset(&parsedMsg, 0, sizeof(parsedMsg));
  memset(&parsedStatus, 0, sizeof(parsedStatus));
  for (uint8_t i = 0; i < pkt.len; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_2, pkt.payload[i], &parsedMsg, &parsedStatus)) {
      if (parsedMsg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&parsedMsg, &ack);
        if ((uint16_t)ack.command == command) return true;
      }
    }
  }
  return false;
}

void purgeQueuedCommandAckForGCS(uint16_t command) {
  uint16_t n = highCount;
  for (uint16_t i = 0; i < n; i++) {
    MavlinkRawPacket pkt = highQueue[highTail];
    popHighRawPacket();
    if (rawPacketContainsCommandAck(pkt, command)) {
      highQueueDrop++;
      rawCommandDrop++;
      continue;
    }
    pushExistingHighRawPacket(pkt);
  }
}

bool enqueueMavlinkHighForGCS(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (len <= RAW_MAVLINK_MAX) return enqueueHighRawPacketBundled(buf, (uint8_t)len);
  oversizedMavlinkDrop++; rawCommandDrop++; return false;
}
bool enqueueMavlinkLowForGCS(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (len <= RAW_MAVLINK_MAX) return enqueueLowRawPacket(buf, (uint8_t)len);
  oversizedMavlinkDrop++; lowQueueDrop++; return false;
}
bool enqueueMavlinkLowForGCSBundled(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (len <= RAW_MAVLINK_MAX) return enqueueLowRawPacketBundled(buf, (uint8_t)len);
  oversizedMavlinkDrop++; lowQueueDrop++; return false;
}
bool isCriticalCalibrationFeedback(const mavlink_message_t &msg) {
  switch (msg.msgid) {
#ifdef MAVLINK_MSG_ID_MAG_CAL_PROGRESS
    case MAVLINK_MSG_ID_MAG_CAL_PROGRESS: return true;
#endif
#ifdef MAVLINK_MSG_ID_MAG_CAL_REPORT
    case MAVLINK_MSG_ID_MAG_CAL_REPORT: return true;
#endif
    default: return false;
  }
}
uint8_t getTargetSysId() {
  return autopilotIdentityLocked ? autopilotTargetSysId : 1;
}

uint8_t getTargetCompId() {
  return autopilotIdentityLocked ? autopilotTargetCompId : MAV_COMP_ID_AUTOPILOT1;
}

static inline bool heartbeatLooksLikeAutopilot(const mavlink_message_t &msg, const mavlink_heartbeat_t &hb) {
  if (hb.autopilot == MAV_AUTOPILOT_INVALID) return false;
  if (hb.type == MAV_TYPE_GCS) return false;
  if (msg.compid == MAV_COMP_ID_AUTOPILOT1) return true;
  return hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA;
}

static inline void lockAutopilotIdentityFromHeartbeat(const mavlink_message_t &msg, const mavlink_heartbeat_t &hb) {
  if (!heartbeatLooksLikeAutopilot(msg, hb)) return;

  autopilotTargetSysId = msg.sysid ? msg.sysid : 1;
  autopilotTargetCompId = msg.compid ? msg.compid : MAV_COMP_ID_AUTOPILOT1;
  autopilotIdentityLocked = true;
  autopilotIdentityLockCount++;

  // Beacon source identity follows the locked autopilot target.  Do not update
  // these fields from arbitrary non-heartbeat messages; command targeting must
  // stay stable while PARAM_SET/Motor Test traffic is in flight.
  pixDataFull.system_id = autopilotTargetSysId;
  pixDataFull.component_id = autopilotTargetCompId;
}

static inline bool shouldForwardRawDiagnostic(unsigned long &lastForwardMs) {
  if (currentSF > 9) return false;
  unsigned long now = millis();
  unsigned long interval = paramSyncActive ? DIAG_RAW_SYNC_INTERVAL_MS : DIAG_RAW_NORMAL_INTERVAL_MS;
  if (now - lastForwardMs < interval) return false;
  lastForwardMs = now;
  return true;
}
bool isPixhawkArmed() { return (pixDataFull.valid_flags & VALID_HEARTBEAT) && (pixDataFull.base_mode & MAV_MODE_FLAG_SAFETY_ARMED); }
void writeMavlinkToPixhawk(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  PixhawkSerial.write(buf, len);
}

static inline bool isValidBatteryVoltageMv(uint16_t voltageMv) {
  return voltageMv != 0 && voltageMv != UINT16_MAX;
}

static inline bool isValidBatteryCurrentCA(int16_t currentCA) {
  return currentCA != -1;
}

static inline bool isValidBatteryRemainingPct(int8_t remainingPct) {
  return remainingPct >= 0 && remainingPct <= 100;
}

void resetBatteryTelemetryDefaults() {
  pixDataFull.voltage_battery = UINT16_MAX;
  pixDataFull.current_battery = -1;
  pixDataFull.battery_remaining = -1;
}

void mergeBatteryTelemetry(uint16_t voltageMv, int16_t currentCA, int8_t remainingPct) {
  bool hasAnyValidField = false;

  if (isValidBatteryVoltageMv(voltageMv)) {
    pixDataFull.voltage_battery = voltageMv;
    hasAnyValidField = true;
  }
  if (isValidBatteryCurrentCA(currentCA)) {
    pixDataFull.current_battery = currentCA;
    hasAnyValidField = true;
  }
  if (isValidBatteryRemainingPct(remainingPct)) {
    pixDataFull.battery_remaining = remainingPct;
    hasAnyValidField = true;
  }

  if (hasAnyValidField) pixDataFull.valid_flags |= VALID_SYS_STATUS;
}

void sendCommandLongToPixhawk(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7) {
  mavlink_message_t outMsg;
  mavlink_command_long_t cmd = {};
  cmd.target_system = getTargetSysId(); cmd.target_component = getTargetCompId();
  cmd.command = command; cmd.confirmation = 0;
  cmd.param1 = p1; cmd.param2 = p2; cmd.param3 = p3; cmd.param4 = p4;
  cmd.param5 = p5; cmd.param6 = p6; cmd.param7 = p7;
  mavlink_msg_command_long_encode(RADIO_SYS_ID, RADIO_COMP_ID, &outMsg, &cmd);
  writeMavlinkToPixhawk(outMsg);
}

void requestMessageInterval(uint32_t messageId, int32_t intervalUs) {
  sendCommandLongToPixhawk(MAV_CMD_SET_MESSAGE_INTERVAL, (float)messageId, (float)intervalUs, 0, 0, 0, 0, 0);
}

void configurePixhawkMessageIntervalsNormal() {
  if (!STREAM_CONFIG_ENABLE) return;
  requestMessageInterval(MAVLINK_MSG_ID_HEARTBEAT,          1000000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE,            200000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_VFR_HUD,             200000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_GPS_RAW_INT,         500000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_SYS_STATUS,          500000L); delay(8);
#ifdef MAVLINK_MSG_ID_BATTERY_STATUS
  requestMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS,     1000000L); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_VIBRATION
  // Request VIBRATION at 5 Hz (200 ms) — compact enough for 109-byte beacon packing
  requestMessageInterval(MAVLINK_MSG_ID_VIBRATION,           200000L); delay(8);
#endif

#if MOTOR_MONITOR_ENABLE
  requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, SERVO_OUTPUT_MONITOR_INTERVAL_US); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS,      RC_CHANNELS_MONITOR_INTERVAL_US); delay(8);
#ifdef MAVLINK_MSG_ID_RC_CHANNELS_RAW
  requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS_RAW,  RC_CHANNELS_MONITOR_INTERVAL_US); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS
  requestMessageInterval(MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, ACTUATOR_MONITOR_INTERVAL_US); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4
  requestMessageInterval(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, ESC_MONITOR_INTERVAL_US); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_STATUS
  requestMessageInterval(MAVLINK_MSG_ID_ESC_STATUS, ESC_MONITOR_INTERVAL_US); delay(8);
#endif
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
  requestMessageInterval(MAVLINK_MSG_ID_EKF_STATUS_REPORT, 500000L); delay(8);
#endif

  streamConfigSentCount++;
}
void configurePixhawkMessageIntervalsParamSync() {
  if (!STREAM_CONFIG_ENABLE) return;
  requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, PARAM_SYNC_STREAM_SLOW_US); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, PARAM_SYNC_STREAM_SLOW_US); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_VFR_HUD, PARAM_SYNC_STREAM_SLOW_US); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_GPS_RAW_INT, PARAM_SYNC_STREAM_SLOW_US); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_SYS_STATUS, PARAM_SYNC_STREAM_SLOW_US); delay(8);
#ifdef MAVLINK_MSG_ID_BATTERY_STATUS
  requestMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, 2000000L); delay(8);
#endif
#if MOTOR_MONITOR_ENABLE
  if (calibrationConfigModeActive()) {
    requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, INTERACTIVE_SERVO_OUTPUT_INTERVAL_US); delay(8);
    requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS, INTERACTIVE_RC_CHANNELS_INTERVAL_US); delay(8);
#ifdef MAVLINK_MSG_ID_RC_CHANNELS_RAW
    requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS_RAW, INTERACTIVE_RC_CHANNELS_INTERVAL_US); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS
    requestMessageInterval(MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, INTERACTIVE_ACTUATOR_INTERVAL_US); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4
    requestMessageInterval(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, INTERACTIVE_ESC_INTERVAL_US); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_STATUS
    requestMessageInterval(MAVLINK_MSG_ID_ESC_STATUS, INTERACTIVE_ESC_INTERVAL_US); delay(8);
#endif
  } else {
    requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, -1); delay(8);
    requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS, -1); delay(8);
#ifdef MAVLINK_MSG_ID_RC_CHANNELS_RAW
    requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS_RAW, -1); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS
    requestMessageInterval(MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, -1); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4
    requestMessageInterval(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, -1); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_STATUS
    requestMessageInterval(MAVLINK_MSG_ID_ESC_STATUS, -1); delay(8);
#endif
  }
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
  requestMessageInterval(MAVLINK_MSG_ID_EKF_STATUS_REPORT, PARAM_SYNC_STREAM_SLOW_US); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_VIBRATION
  requestMessageInterval(MAVLINK_MSG_ID_VIBRATION, 2000000L); delay(8);
#endif
  streamConfigSentCount++;
}
void configurePixhawkMessageIntervalsInteractiveSetup() {
  if (!STREAM_CONFIG_ENABLE) return;
  requestMessageInterval(MAVLINK_MSG_ID_HEARTBEAT, 2000000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, 2000000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2000000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_VFR_HUD, 2000000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_GPS_RAW_INT, 2000000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_SYS_STATUS, 2000000L); delay(8);
#ifdef MAVLINK_MSG_ID_BATTERY_STATUS
  requestMessageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, 2000000L); delay(8);
#endif
#if MOTOR_MONITOR_ENABLE
  requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 2000000L); delay(8);
  requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS, 1000000L); delay(8);
#ifdef MAVLINK_MSG_ID_RC_CHANNELS_RAW
  requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS_RAW, 1000000L); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS
  requestMessageInterval(MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, 5000000L); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4
  requestMessageInterval(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, 5000000L); delay(8);
#endif
#ifdef MAVLINK_MSG_ID_ESC_STATUS
  requestMessageInterval(MAVLINK_MSG_ID_ESC_STATUS, 5000000L); delay(8);
#endif
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
  requestMessageInterval(MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5000000L); delay(8);
#endif
  streamConfigSentCount++;
}
void enterParamSyncStreamMode() {
#if PARAM_SYNC_STREAM_SLOWDOWN_ENABLE
  if (!STREAM_CONFIG_ENABLE) return;
  if (paramSyncStreamSlowed) return;
  if (calibrationConfigModeActive()) configurePixhawkMessageIntervalsInteractiveSetup();
  else configurePixhawkMessageIntervalsParamSync();
  paramSyncStreamSlowed = true; streamConfigDone = true;
#endif
}
void exitParamSyncStreamMode() {
#if PARAM_SYNC_STREAM_SLOWDOWN_ENABLE
  if (!STREAM_CONFIG_ENABLE) return;
  if (!paramSyncStreamSlowed) return;
  configurePixhawkMessageIntervalsNormal();
  paramSyncStreamSlowed = false; streamConfigDone = false; streamConfigRetry = 0; lastStreamConfigMs = 0;
#endif
}
void updatePixhawkStreamConfig() {
  if (!STREAM_CONFIG_ENABLE) return;
  if (activeConfigOrParamMode()) return;
  if (streamConfigDone) return;
  unsigned long now = millis();
  if (streamConfigRetry == 0 || now - lastStreamConfigMs >= STREAM_CONFIG_RETRY_MS) {
    configurePixhawkMessageIntervalsNormal();
    lastStreamConfigMs = now; streamConfigRetry++; streamConfigRetryCount++;
    if (streamConfigRetry >= STREAM_CONFIG_MAX_RETRY) streamConfigDone = true;
  }
}
unsigned long syntheticGcsHeartbeatValidMsForSF(uint8_t sf) {
  if (sf >= 12) return SYNTHETIC_GCS_HEARTBEAT_VALID_MS_SF12;
  if (sf == 11) return SYNTHETIC_GCS_HEARTBEAT_VALID_MS_SF11;
  if (sf == 10) return SYNTHETIC_GCS_HEARTBEAT_VALID_MS_SF10;
  return SYNTHETIC_GCS_HEARTBEAT_VALID_MS;
}

void sendSyntheticGCSHeartbeatToPixhawkIfNeeded() {
#if SYNTHETIC_GCS_HEARTBEAT_ENABLE
  unsigned long now = millis();
  if (lastGcsContactMs == 0) return;
  if (now - lastGcsContactMs > syntheticGcsHeartbeatValidMsForSF(currentSF)) return;
  if (now - lastSyntheticGcsHeartbeatMs < SYNTHETIC_GCS_HEARTBEAT_INTERVAL_MS) return;
  mavlink_message_t hb;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(SYNTHETIC_GCS_SYS_ID, SYNTHETIC_GCS_COMP_ID, &hb, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &hb);
  PixhawkSerial.write(buf, len);
  lastSyntheticGcsHeartbeatMs = now;
#endif
}

void recordAckResultForTelemetryMeta(bool ackOK) {
  // Keep the requested link metrics real and useful:
  // cntACK = number of successful ACK/downlink contacts in last 10 ACK opportunities.
  ackHist[histIdx] = ackOK ? 1 : 0;
  histIdx = (histIdx + 1) % 10;
  int sum = 0;
  for (uint8_t i = 0; i < 10; i++) sum += ackHist[i];
  cntACK = constrain(sum, 0, 10);

  if (ackOK) {
    if (successStreak < 65535) successStreak++;
    failStreak = 0;
  } else {
    if (failStreak < 65535) failStreak++;
    successStreak = 0;
  }
}

void sendSetModeToPixhawk(uint32_t customMode) {
  mavlink_message_t outMsg;
  mavlink_msg_set_mode_pack(RADIO_SYS_ID, RADIO_COMP_ID, &outMsg,
                            getTargetSysId(),
                            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            customMode);
  writeMavlinkToPixhawk(outMsg);
}

void sendSafetyRTLToPixhawk() {
  if (!isPixhawkArmed()) return;
  sendSetModeToPixhawk(ARDUPILOT_COPTER_MODE_RTL);
  sendCommandLongToPixhawk(MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,0,0,0,0,0,0);
}

void sendSafetyLandToPixhawk() {
  if (!isPixhawkArmed()) return;
  sendSetModeToPixhawk(ARDUPILOT_COPTER_MODE_LAND);
  sendCommandLongToPixhawk(MAV_CMD_NAV_LAND, 0,0,0,0,0,0,0);
}

void sendSafetyBrakeToPixhawk() {
  if (!isPixhawkArmed()) return;
  sendSetModeToPixhawk(ARDUPILOT_COPTER_MODE_BRAKE);
}

void sendBridgeFailsafeActionToPixhawk() {
#if BRIDGE_FAILSAFE_ENABLE
  if (!isPixhawkArmed()) return;
  #if BRIDGE_FAILSAFE_ACTION == BRIDGE_FAILSAFE_ACTION_LAND
    sendSafetyLandToPixhawk();
  #elif BRIDGE_FAILSAFE_ACTION == BRIDGE_FAILSAFE_ACTION_BRAKE
    sendSafetyBrakeToPixhawk();
  #else
    sendSafetyRTLToPixhawk();
  #endif
#endif
}

void handleLinkHealth(bool ackOK) {
  recordAckResultForTelemetryMeta(ackOK);

  if (ackOK) {
    consecutiveLinkFails = 0;
    lastAckOkMs = millis();
    lastGcsContactMs = lastAckOkMs;
    return;
  }

  if (consecutiveLinkFails < 255) consecutiveLinkFails++;
  if (!ENABLE_SAFETY_RTL) return;
  if (consecutiveLinkFails < LINK_FAIL_RTL_THRESHOLD) return;

  unsigned long now = millis();
  bool timeoutExceeded = (lastAckOkMs == 0) || (now - lastAckOkMs >= BRIDGE_FAILSAFE_TIMEOUT_MS);
  if (!timeoutExceeded) return;
  if (now - lastRtlCommandMs < RTL_REPEAT_INTERVAL_MS) return;

  lastRtlCommandMs = now;
  sendBridgeFailsafeActionToPixhawk();
}

// ================= M-SADR DISABLED (Static SF mode) =================
int findSFIndex(int sf) { return 0; }
int selectSFByMaxProbability() { return currentSF; }
float calc_b() { return 0.05f; }
void updateHist(int ack) {}
void updateP(int sfIdx, int rack) {}
void updateHigherSFProbabilitiesAfterFailure() {}
void updateMSADRCore(bool ackOK) { handleLinkHealth(ackOK); }
void updateMSADRNormal(bool ackOK) { handleLinkHealth(ackOK); }
void updateMSADR(bool ackOK) { handleLinkHealth(ackOK); }
// ========================================================

void updateConfigStuckGuards() {
  unsigned long now = millis();
  if (configPending && configPendingSinceMs > 0 && now - configPendingSinceMs > CONFIG_STUCK_CANCEL_MS) {
    configPending = false; configPendingSinceMs = 0; configStuckCancelCount++; softRecoverRadio(false);
  }
  if (scheduledConfig && scheduledConfigSinceMs > 0 && now - scheduledConfigSinceMs > CONFIG_STUCK_CANCEL_MS) {
    scheduledConfig = false; scheduledConfigSinceMs = 0; configStuckCancelCount++; softRecoverRadio(false);
  }
}
void updateRadioSoftRecovery() {
  unsigned long now = millis();
  if (now - lastRadioRecoveryMs < RADIO_SOFT_RECOVERY_MIN_GAP_MS) return;
  if (lastAckOkMs == 0) return;
  if (now - lastAckOkMs > RADIO_SOFT_RECOVERY_MS && consecutiveLinkFails >= 5) softRecoverRadio(true);
}

bool isSetupCommand(uint16_t command) {
  return bridgeIsSetupConfigCommand(command);
}

bool isFlightActionCommandId(uint16_t command) {
  /*
   * Blocking Mission Planner command classifier.
   *
   * The name is historical: this is now the set of commands that must interrupt
   * parameter bulk transfer because Mission Planner waits for real vehicle
   * feedback. Motor Test (209) is included to fix Optional Hardware -> Motor Test
   * "failed to communicate with autopilot" failures.
   */
  return bridgeIsFlightActionCommand(command);
}


bool isRawMissionPlannerControlMessage(const mavlink_message_t &msg) {
  /*
   * Raw downlink command priority detector.
   * Compact commands already call beginFlightCommandPriorityMode() in
   * handleCompactCommandFromGCS(). This helper covers raw MAVLink packets at
   * SF7 and PARAM_SET writes at all SFs.
   */
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) return true;
#ifdef MAVLINK_MSG_ID_PARAM_EXT_SET
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_EXT_SET) return true;
#endif
  if (msg.msgid == MAVLINK_MSG_ID_SET_MODE ||
      msg.msgid == MAVLINK_MSG_ID_MISSION_SET_CURRENT ||
      msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL ||
      msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
    return true;
  }
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    uint16_t command = (uint16_t)cmd.command;
    return isFlightActionCommandId(command) || isSetupCommand(command);
  }
#ifdef MAVLINK_MSG_ID_COMMAND_INT
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    mavlink_command_int_t cmd;
    mavlink_msg_command_int_decode(&msg, &cmd);
    uint16_t command = (uint16_t)cmd.command;
    return isFlightActionCommandId(command) || isSetupCommand(command);
  }
#endif
  return false;
}

void beginFlightCommandPriorityMode() {
  flightCommandRxCount++;
  commandModeUntilMs = millis() + FLIGHT_COMMAND_HOLD_MS;
  if (paramSyncActive) abortParamSyncRecovery("flight_command_priority");
  // Prioritizes user commands ahead of queued telemetry packets:
  // stale high-priority feedback from an older command. Real FC feedback for the
  // new command will be queued immediately after the command reaches the FC.
  flushHighQueueForRecovery();
  flushLowQueue();
  flushParamBulkQueue();
  linkMode = LINK_MODE_COMMAND;
}

void updateFlightCommandPriorityMode() {
  if (commandModeUntilMs == 0) return;
  if (millis() <= commandModeUntilMs) { linkMode = LINK_MODE_COMMAND; return; }
  commandModeUntilMs = 0;
  if (!paramSyncActive && !calibrationConfigModeActive()) linkMode = LINK_MODE_NORMAL;
}

// ================= Spreading Factor Runtime Handlers =================
void handleParamSet(const mavlink_param_set_t &ps) {
  char param_id[17];
  copyParamId(param_id, ps.param_id);
  if (strcmp(param_id, "STATIC_SF") == 0) {
    uint8_t newSF = (uint8_t)ps.param_value;
    if (newSF >= SF_MIN && newSF <= SF_MAX) {
      // UAV is the PHY master. Runtime SF changes must be negotiated via
      // CONFIG_PROPOSE/CONFIG_ACK on the old SF so that GCS also migrates to the new SF.
      EEPROM.write(EEPROM_SF_ADDR, newSF);
      EEPROM.commit();
      proposeConfig(newSF, currentTP);
      // Send PARAM_VALUE packet to confirm SF proposal reception.
      mavlink_message_t msg;
      mavlink_param_value_t pv;
      pv.param_value = newSF;
      pv.param_count = 1;
      pv.param_index = 0;
      copyParamId16(pv.param_id, "STATIC_SF");
      pv.param_type = MAV_PARAM_TYPE_UINT8;
      mavlink_msg_param_value_encode(RADIO_SYS_ID, RADIO_COMP_ID, &msg, &pv);
      enqueueMavlinkLowForGCS(msg);
    }
  }
}
// =============================================================

void inspectRawCommandFromGCS(const uint8_t *payload, uint8_t len) {
  mavlink_message_t parsedMsg;
  mavlink_status_t parsedStatus;
  memset(&parsedMsg, 0, sizeof(parsedMsg));
  memset(&parsedStatus, 0, sizeof(parsedStatus));

  for (uint8_t i = 0; i < len; i++) {
    if (!mavlink_parse_char(MAVLINK_COMM_1, payload[i], &parsedMsg, &parsedStatus)) continue;

    if (isRawMissionPlannerControlMessage(parsedMsg)) {
      /*
       * Any user-blocking raw command/write preempts full parameter sync.
       * This prevents PARAM_SET and Motor Test feedback from being delayed behind
       * PARAM_BULK. The raw packet is still forwarded to Pixhawk below unless a
       * later branch explicitly suppresses it.
       */
      beginFlightCommandPriorityMode();
    }

    if (parsedMsg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
      // Critical fix: never forward Mission Planner's full-list request to Pixhawk.
      // ArduPilot would otherwise dump 1000+ PARAM_VALUE messages over UART and overflow LoRa queues.
      suppressCurrentRawToPixhawk = true;
      if (fullParamSyncAllowedAtCurrentSF()) {
        startAsyncParamSyncProxy(parsedMsg.sysid, parsedMsg.compid);
      } else {
        fullParamSyncBlockedHighSfCount++;
        abortParamSyncRecovery("block_full_param_high_sf");
        enqueueBridgeStatusText(MAV_SEVERITY_WARNING, "Param get blocked: use SF7-SF9");
      }
      continue;
    }

    if (parsedMsg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
      mavlink_param_request_read_t pr;
      mavlink_msg_param_request_read_decode(&parsedMsg, &pr);
      char id[17];
      copyParamId(id, pr.param_id);

      if (highSfParamTrafficBlocked()) {
        fullParamSyncBlockedHighSfCount++;
        suppressCurrentRawToPixhawk = true;
        abortParamSyncRecovery("block_param_read_high_sf");
        enqueueBridgeStatusText(MAV_SEVERITY_WARNING, "Param get blocked: use SF7-SF9");
      } else if (isAsyncParamSyncActive && !isInteractiveSetupParamId(id)) {
        /*
         * Control-plane isolation during full async sync:
         * Mission Planner may issue PARAM_REQUEST_READ retries/gap requests while
         * it is waiting for x/y progress. Letting those raw reads bypass the proxy
         * or mutate currentAsyncParamIndex creates race conditions:
         *   - stale PARAM_VALUE arrives after cursor jump,
         *   - async filter sees param_index mismatch,
         *   - paramValueDrop rises and sync slows down.
         *
         * Policy:
         *   - suppress generic MP PARAM_REQUEST_READ during active bulk sync;
         *   - if MP asks exactly for the current index, treat it as a retry hint;
         *   - never rewind or jump the main cursor from an external read.
         */
        suppressCurrentRawToPixhawk = true;
        if (pr.param_index >= 0 && (uint16_t)pr.param_index == currentAsyncParamIndex) {
          asyncParamWaitingForValue = false;
          asyncParamLastRequestMs = 0;
        }
      } else {
        if (isInteractiveSetupParamId(id)) startCalibrationConfigMode();
        else if (fullParamSyncAllowedAtCurrentSF()) startParamSync();
      }
      continue;
    }

    if (parsedMsg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
      mavlink_param_set_t ps;
      mavlink_msg_param_set_decode(&parsedMsg, &ps);
      copyParamId(expectedParamId, ps.param_id);
      paramWriteAckExpected = true;
      paramWriteAckUntilMs = millis() + PARAM_WRITE_ACK_WINDOW_MS;
      if (isInteractiveSetupParamId(expectedParamId)) startCalibrationConfigMode();
      handleParamSet(ps);
      continue;
    }

    if (parsedMsg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
      mavlink_command_long_t cmd;
      mavlink_msg_command_long_decode(&parsedMsg, &cmd);
      if (isFlightActionCommandId((uint16_t)cmd.command)) beginFlightCommandPriorityMode();
      if ((uint16_t)cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
        purgeQueuedCommandAckForGCS(MAV_CMD_COMPONENT_ARM_DISARM);
        enqueueBridgeStatusText(MAV_SEVERITY_NOTICE, "ARM command forwarded to FC; battery failsafe may disarm");
      }
      if (isSetupCommand((uint16_t)cmd.command)) {
        calCommandRxCount++;
        startCalibrationConfigMode();
      }
      continue;
    }

#ifdef MAVLINK_MSG_ID_COMMAND_INT
    if (parsedMsg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
      mavlink_command_int_t cmd;
      mavlink_msg_command_int_decode(&parsedMsg, &cmd);
      if (isFlightActionCommandId((uint16_t)cmd.command)) beginFlightCommandPriorityMode();
      if (isSetupCommand((uint16_t)cmd.command)) {
        calCommandRxCount++;
        startCalibrationConfigMode();
      }
      continue;
    }
#endif

#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST
    if (parsedMsg.msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST) {
      // MAVLink PARAM_EXT full-list is not proxied here. Suppress it to avoid another full dump path.
      suppressCurrentRawToPixhawk = true;
      fullParamSyncBlockedHighSfCount++;
      enqueueBridgeStatusText(MAV_SEVERITY_WARNING, "PARAM_EXT full sync unsupported on LoRa bridge");
      continue;
    }
#endif

#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
    if (parsedMsg.msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ) {
      mavlink_param_ext_request_read_t pr;
      mavlink_msg_param_ext_request_read_decode(&parsedMsg, &pr);
      char id[17];
      copyParamId(id, pr.param_id);
      if (highSfParamTrafficBlocked()) {
        fullParamSyncBlockedHighSfCount++;
        suppressCurrentRawToPixhawk = true;
        abortParamSyncRecovery("block_param_ext_read_high_sf");
        enqueueBridgeStatusText(MAV_SEVERITY_WARNING, "Param get blocked: use SF7-SF9");
      } else {
        if (isInteractiveSetupParamId(id)) startCalibrationConfigMode();
        else if (fullParamSyncAllowedAtCurrentSF()) startParamSync();
      }
      continue;
    }
#endif

#ifdef MAVLINK_MSG_ID_PARAM_EXT_SET
    if (parsedMsg.msgid == MAVLINK_MSG_ID_PARAM_EXT_SET) {
      mavlink_param_ext_set_t ps;
      mavlink_msg_param_ext_set_decode(&parsedMsg, &ps);
      copyParamId(expectedParamExtId, ps.param_id);
      paramExtWriteAckExpected = true;
      paramExtWriteAckUntilMs = millis() + PARAM_EXT_WRITE_ACK_WINDOW_MS;
      if (isInteractiveSetupParamId(expectedParamExtId)) startCalibrationConfigMode();
      continue;
    }
#endif
  }
}

float compactFromX1000(int32_t v) { return ((float)v) / COMPACT_CMD_SCALE_1000; }
float compactFromX1e7(int32_t v) { return ((float)v) / COMPACT_CMD_SCALE_1E7; }

bool handleCompactCommandFromGCS(const uint8_t *buf, size_t len) {
  if (len < sizeof(CompactCommandBasePacket)) return false;
  const CompactCommandBasePacket *base = (const CompactCommandBasePacket *)buf;
  if (!acceptRollingCompactSeqFromGCS(base->seq)) return false;
  mavlink_message_t outMsg;
  memset(&outMsg, 0, sizeof(outMsg));

  if (base->kind == COMPACT_CMD_KIND_ARM_DISARM) {
    if (len != sizeof(CompactArmDisarmPacket)) return false;
    CompactArmDisarmPacket pkt; memcpy(&pkt, buf, sizeof(pkt));
    beginFlightCommandPriorityMode();
    purgeQueuedCommandAckForGCS(MAV_CMD_COMPONENT_ARM_DISARM);
    enqueueBridgeStatusText(MAV_SEVERITY_NOTICE, "ARM command forwarded to FC; battery failsafe may disarm");
    mavlink_command_long_t cmd = {};
    cmd.target_system = pkt.target_system ? pkt.target_system : getTargetSysId();
    cmd.target_component = pkt.target_component ? pkt.target_component : getTargetCompId();
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = pkt.arm ? 1.0f : 0.0f;
    cmd.param2 = compactFromX1000(pkt.param2_x1000);
    mavlink_msg_command_long_encode(RADIO_SYS_ID, RADIO_COMP_ID, &outMsg, &cmd);
    writeMavlinkToPixhawk(outMsg);
    flightCommandRxCount++;
    return true;
  }

  if (base->kind == COMPACT_CMD_KIND_SET_MODE) {
    if (len != sizeof(CompactSetModePacket)) return false;
    CompactSetModePacket pkt; memcpy(&pkt, buf, sizeof(pkt));
    beginFlightCommandPriorityMode();
    mavlink_set_mode_t sm = {};
    sm.target_system = pkt.target_system ? pkt.target_system : getTargetSysId();
    sm.base_mode = pkt.base_mode;
    sm.custom_mode = pkt.custom_mode;
    mavlink_msg_set_mode_encode(RADIO_SYS_ID, RADIO_COMP_ID, &outMsg, &sm);
    writeMavlinkToPixhawk(outMsg);
    flightCommandRxCount++;
    return true;
  }

  if (base->kind == COMPACT_CMD_KIND_COMMAND_LONG) {
    if (len != sizeof(CompactCommandLongPacket)) return false;
    CompactCommandLongPacket pkt; memcpy(&pkt, buf, sizeof(pkt));
    beginFlightCommandPriorityMode();
    if (isSetupCommand(pkt.command)) { calCommandRxCount++; startCalibrationConfigMode(); }
    mavlink_command_long_t cmd = {};
    cmd.target_system = pkt.target_system ? pkt.target_system : getTargetSysId();
    cmd.target_component = pkt.target_component ? pkt.target_component : getTargetCompId();
    cmd.command = pkt.command;
    cmd.confirmation = pkt.confirmation;
    cmd.param1 = compactFromX1000(pkt.p1_x1000);
    cmd.param2 = compactFromX1000(pkt.p2_x1000);
    cmd.param3 = compactFromX1000(pkt.p3_x1000);
    cmd.param4 = compactFromX1000(pkt.p4_x1000);
    cmd.param5 = compactFromX1e7(pkt.p5_x1e7);
    cmd.param6 = compactFromX1e7(pkt.p6_x1e7);
    cmd.param7 = compactFromX1000(pkt.p7_x1000);
    mavlink_msg_command_long_encode(RADIO_SYS_ID, RADIO_COMP_ID, &outMsg, &cmd);
    writeMavlinkToPixhawk(outMsg);
    flightCommandRxCount++;
    return true;
  }

  return false;
}

void updateTelemetryFromPixhawk(const mavlink_message_t &msg) {
  lastPixhawkMavlinkMs = millis();
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      lastPixhawkHeartbeatMs = millis();
      mavlink_heartbeat_t hb; mavlink_msg_heartbeat_decode(&msg, &hb);
      lockAutopilotIdentityFromHeartbeat(msg, hb);
      pixDataFull.mav_type = hb.type; pixDataFull.autopilot = hb.autopilot;
      pixDataFull.base_mode = hb.base_mode; pixDataFull.custom_mode = hb.custom_mode;
      pixDataFull.system_status = hb.system_status; pixDataFull.valid_flags |= VALID_HEARTBEAT;
      break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t at; mavlink_msg_attitude_decode(&msg, &at);
      pixDataFull.roll = at.roll; pixDataFull.pitch = at.pitch; pixDataFull.yaw = at.yaw;
      pixDataFull.rollspeed = at.rollspeed; pixDataFull.pitchspeed = at.pitchspeed; pixDataFull.yawspeed = at.yawspeed;
      pixDataFull.valid_flags |= VALID_ATTITUDE;
      break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t gp; mavlink_msg_global_position_int_decode(&msg, &gp);
      pixDataFull.lat = gp.lat; pixDataFull.lon = gp.lon; pixDataFull.alt_mm = gp.alt;
      pixDataFull.relative_alt_mm = gp.relative_alt; pixDataFull.vx = gp.vx; pixDataFull.vy = gp.vy;
      pixDataFull.vz = gp.vz; pixDataFull.hdg = gp.hdg; pixDataFull.valid_flags |= VALID_GLOBAL_POS;
      // vx/vy/vz present whenever GLOBAL_POSITION_INT is valid
      pixDataFull.valid_flags |= VALID_LOCAL_VEL;
      break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_gps_raw_int_t gps; mavlink_msg_gps_raw_int_decode(&msg, &gps);
      pixDataFull.gps.time_usec = gps.time_usec; pixDataFull.gps.fix_type = gps.fix_type;
      pixDataFull.gps.lat = gps.lat; pixDataFull.gps.lon = gps.lon; pixDataFull.gps.alt_mm = gps.alt;
      pixDataFull.gps.eph = gps.eph; pixDataFull.gps.epv = gps.epv; pixDataFull.gps.vel = gps.vel;
      pixDataFull.gps.cog = gps.cog; pixDataFull.gps.satellites_visible = gps.satellites_visible;
      pixDataFull.valid_flags |= VALID_GPS_RAW;
      break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t hud; mavlink_msg_vfr_hud_decode(&msg, &hud);
      pixDataFull.airspeed = hud.airspeed; pixDataFull.groundspeed = hud.groundspeed;
      pixDataFull.heading = hud.heading; pixDataFull.throttle = hud.throttle; pixDataFull.climb = hud.climb;
      pixDataFull.valid_flags |= VALID_VFR_HUD;
      break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t st; mavlink_msg_sys_status_decode(&msg, &st);
      mergeBatteryTelemetry(st.voltage_battery, st.current_battery, st.battery_remaining);
      break;
    }
#ifdef MAVLINK_MSG_ID_BATTERY_STATUS
    case MAVLINK_MSG_ID_BATTERY_STATUS: {
      mavlink_battery_status_t bs;
      mavlink_msg_battery_status_decode(&msg, &bs);

      uint32_t totalMv = 0;
      uint16_t firstMv = 0;
      uint8_t validCells = 0;
      for (uint8_t i = 0; i < 10; i++) {
        if (bs.voltages[i] == UINT16_MAX || bs.voltages[i] == 0) continue;
        if (validCells == 0) firstMv = bs.voltages[i];
        totalMv += bs.voltages[i];
        validCells++;
      }

      uint16_t packVoltageMv = UINT16_MAX;
      if (validCells == 1) packVoltageMv = firstMv;
      else if (validCells > 1) packVoltageMv = (totalMv > 65534UL) ? 65534U : (uint16_t)totalMv;
      mergeBatteryTelemetry(packVoltageMv, bs.current_battery, bs.battery_remaining);
      break;
    }
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
      mavlink_ekf_status_report_t ekf; mavlink_msg_ekf_status_report_decode(&msg, &ekf);
      pixDataFull.ekf_flags = ekf.flags; pixDataFull.valid_flags |= VALID_EKF;
      break;
    }
#endif
#ifdef MAVLINK_MSG_ID_VIBRATION
    // MAVLink #241 VIBRATION: vibration levels and accel clipping counts
    case MAVLINK_MSG_ID_VIBRATION: {
      mavlink_vibration_t vib; mavlink_msg_vibration_decode(&msg, &vib);
      pixDataFull.vibe_x       = vib.vibration_x;
      pixDataFull.vibe_y       = vib.vibration_y;
      pixDataFull.vibe_z       = vib.vibration_z;
      pixDataFull.accel_clip_0 = vib.clipping_0;
      pixDataFull.accel_clip_1 = vib.clipping_1;
      pixDataFull.accel_clip_2 = vib.clipping_2;
      pixDataFull.valid_flags2 |= VALID2_VIBRATION;
      break;
    }
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS
    // MAVLink #65 RC_CHANNELS: radio control channel input values
    case MAVLINK_MSG_ID_RC_CHANNELS: {
      mavlink_rc_channels_t rcc; mavlink_msg_rc_channels_decode(&msg, &rcc);
      pixDataFull.rc_channels[0]  = rcc.chan1_raw;  pixDataFull.rc_channels[1]  = rcc.chan2_raw;
      pixDataFull.rc_channels[2]  = rcc.chan3_raw;  pixDataFull.rc_channels[3]  = rcc.chan4_raw;
      pixDataFull.rc_channels[4]  = rcc.chan5_raw;  pixDataFull.rc_channels[5]  = rcc.chan6_raw;
      pixDataFull.rc_channels[6]  = rcc.chan7_raw;  pixDataFull.rc_channels[7]  = rcc.chan8_raw;
      pixDataFull.rc_channels[8]  = rcc.chan9_raw;  pixDataFull.rc_channels[9]  = rcc.chan10_raw;
      pixDataFull.rc_channels[10] = rcc.chan11_raw; pixDataFull.rc_channels[11] = rcc.chan12_raw;
      pixDataFull.rc_channels[12] = rcc.chan13_raw; pixDataFull.rc_channels[13] = rcc.chan14_raw;
      pixDataFull.rc_channels[14] = rcc.chan15_raw; pixDataFull.rc_channels[15] = rcc.chan16_raw;
      pixDataFull.valid_flags2 |= VALID2_RC_CHANNELS;
      break;
    }
#endif
    default: break;
  }
}

void enqueuePixhawkMavlinkIfNeeded(const mavlink_message_t &msg) {
  updateParamWriteAckState();
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) { calCommandAckRxCount++; enqueueMavlinkHighForGCS(msg); return; }
  if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) { calStatustextRxCount++; enqueueMavlinkHighForGCS(msg); return; }
  // Low-latency feedback: immediately forward flight controller responses following a command.
  // the next real FC HEARTBEAT/SYS_STATUS through the high queue. This makes
  // Mission Planner update mode/armed/battery state quickly without synthetic UI.
  if (commandModeActive()) {
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT || msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
      enqueueMavlinkHighForGCS(msg);
      return;
    }
  }
#ifdef MAVLINK_MSG_ID_MAG_CAL_PROGRESS
  if (msg.msgid == MAVLINK_MSG_ID_MAG_CAL_PROGRESS) {
    magCalProgressRxCount++;
    calConfigUntilMs = millis() + CAL_CONFIG_HOLD_MS;
    calConfigActive = true;
    linkMode = LINK_MODE_CALIBRATION;
    cacheMagCalProgressForGCS(msg);
    return;
  }
#endif
#ifdef MAVLINK_MSG_ID_MAG_CAL_REPORT
  if (msg.msgid == MAVLINK_MSG_ID_MAG_CAL_REPORT) { magCalReportRxCount++; calConfigUntilMs = millis() + CAL_CONFIG_HOLD_MS; enqueueMavlinkHighForGCS(msg); return; }
#endif
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_MISSION_ACK:
    case MAVLINK_MSG_ID_MISSION_COUNT:
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    case MAVLINK_MSG_ID_MISSION_ITEM:
#ifdef MAVLINK_MSG_ID_MISSION_REQUEST_INT
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
#endif
#ifdef MAVLINK_MSG_ID_MISSION_ITEM_INT
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
#endif
#ifdef MAVLINK_MSG_ID_MISSION_ITEM_REACHED
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
#endif
#ifdef MAVLINK_MSG_ID_AUTOPILOT_VERSION
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
#endif
#ifdef MAVLINK_MSG_ID_HOME_POSITION
    case MAVLINK_MSG_ID_HOME_POSITION:
#endif
      enqueueMavlinkHighForGCS(msg); break;
    case MAVLINK_MSG_ID_PARAM_VALUE: {
      mavlink_param_value_t pv;
      mavlink_msg_param_value_decode(&msg, &pv);

      char id[17];
      copyParamId(id, pv.param_id);

      bool isParamWriteAck = paramWriteAckExpected && sameParamId(id, expectedParamId);
      bool isInteractiveParamValue = calibrationConfigModeActive() &&
                                     linkSfAllowsParamGet((uint8_t)currentSF) &&
                                     isInteractiveSetupParamId(id);

      if (!linkSfAllowsParamGet((uint8_t)currentSF) && !isParamWriteAck) {
        // Hard block stale/full-sync PARAM_VALUE leakage at SF10-SF12.
        paramValueDrop++;
        break;
      }

      // PARAM_SET confirmation must remain high priority and must bypass bulk sync.
      if (isParamWriteAck) {
        lastParamValueMs = millis();
        paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
        lastParamIndex = pv.param_index;
        lastParamCount = pv.param_count;
        if (!enqueueMavlinkHighForGCS(msg)) paramValueDrop++;
        paramWriteAckExpected = false;
        expectedParamId[0] = 0;
        break;
      }

      // Interactive setup params must not be delayed behind bulk sync.
      if (isInteractiveParamValue) {
        lastParamValueMs = millis();
        paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
        lastParamIndex = pv.param_index;
        lastParamCount = pv.param_count;
        if (!enqueueMavlinkHighForGCS(msg)) paramValueDrop++;
        break;
      }

      if (isAsyncParamSyncActive) {
        if (pv.param_count > 0) asyncParamTotalCount = pv.param_count;

        if (pv.param_index < 0 || (uint16_t)pv.param_index != currentAsyncParamIndex) {
          // Ignore stale/out-of-order PARAM_VALUE. This is the safety wall that prevents
          // accidental Pixhawk streaming from corrupting Mission Planner's parameter table.
          paramValueDrop++;
          asyncParamWaitingForValue = false;
          asyncParamLastRequestMs = 0;
          break;
        }

        if (enqueueParamValueBulkForGCS(msg)) {
          lastParamValueMs = millis();
          paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
          lastParamIndex = pv.param_index;
          lastParamCount = pv.param_count;

          asyncParamLastValueMs = lastParamValueMs;
          asyncParamWaitingForValue = false;
          currentAsyncParamIndex++;

          if (asyncParamTotalCount > 0 && currentAsyncParamIndex >= asyncParamTotalCount) {
            isAsyncParamSyncActive = false;
            asyncParamWaitingForValue = false;
            asyncParamPollDoneCount++;
          }
        } else {
          // Queue failed. Keep the same index and retry after queues/radio clear.
          paramValueDrop++;
          asyncParamWaitingForValue = false;
          asyncParamLastRequestMs = 0;
        }
        break;
      }

      // Fallback path for individual PARAM_REQUEST_READ not belonging to bulk sync.
      lastParamValueMs = millis();
      paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
      lastParamIndex = pv.param_index;
      lastParamCount = pv.param_count;
      if (paramSyncActive) {
        if (canAcceptParamValueToLowQueue()) {
          if (!enqueueParamValueBulkForGCS(msg)) paramValueDrop++;
        }
      } else {
        if (!enqueueMavlinkLowForGCS(msg)) paramValueDrop++;
      }
      break;
    }
#ifdef MAVLINK_MSG_ID_PARAM_EXT_VALUE
    case MAVLINK_MSG_ID_PARAM_EXT_VALUE: {
      if (!linkSfAllowsParamGet((uint8_t)currentSF) && !paramExtWriteAckExpected) {
        paramValueDrop++;
        break;
      }
      lastParamValueMs = millis(); paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
      if (!(paramSyncActive ? enqueueMavlinkLowForGCSBundled(msg) : enqueueMavlinkLowForGCS(msg))) paramValueDrop++;
      break;
    }
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_ACK
    case MAVLINK_MSG_ID_PARAM_EXT_ACK:
      enqueueMavlinkHighForGCS(msg); paramExtWriteAckExpected = false; expectedParamExtId[0] = 0;
      break;
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS
    case MAVLINK_MSG_ID_RC_CHANNELS:
      if (calibrationConfigModeActive()) enqueueMavlinkHighForGCS(msg);
      else if (!paramSyncActive && currentSF <= 9) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS_RAW
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
      if (calibrationConfigModeActive()) enqueueMavlinkHighForGCS(msg);
      else if (!paramSyncActive && currentSF <= 9) enqueueMavlinkHighForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
      if (calibrationConfigModeActive()) enqueueMavlinkHighForGCS(msg);
      else if (!paramSyncActive && currentSF <= 9) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#if MOTOR_MONITOR_ENABLE
#ifdef MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS
    case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS:
      if (!activeConfigOrParamMode() && currentSF <= 8) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4
    case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4:
      if (!activeConfigOrParamMode() && currentSF <= 8) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_ESC_STATUS
    case MAVLINK_MSG_ID_ESC_STATUS:
      if (!activeConfigOrParamMode() && currentSF <= 8) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#endif

#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
      // Keep Mission Planner's EKF Status window alive during full sync.
      // The beacon carries flags every cycle, while this raw passthrough carries
      // variance fields at a bounded rate.  This avoids the old all-or-nothing
      // behavior where EKF diagnostics disappeared during Getting Params.
      if (shouldForwardRawDiagnostic(lastRawEkfForwardMs)) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_VIBRATION
    case MAVLINK_MSG_ID_VIBRATION:
      // VIBRATION is the correct source for physical shake/clip diagnostics.
      // Forward it at a bounded rate even during param sync so FC vibration tests
      // are observable without bloating the compact telemetry beacon.
      if (shouldForwardRawDiagnostic(lastRawVibrationForwardMs)) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_BATTERY_STATUS
    case MAVLINK_MSG_ID_BATTERY_STATUS:
      if (!paramSyncActive && currentSF <= 9) enqueueMavlinkLowForGCS(msg);
      break;
#endif

#ifdef MAVLINK_MSG_ID_LOG_ENTRY
    case MAVLINK_MSG_ID_LOG_ENTRY:
      // DataFlash log list metadata. SF7-SF9 only; SF10-SF12 are intentionally not bulk-log links.
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkHighForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_LOG_DATA
    case MAVLINK_MSG_ID_LOG_DATA:
      // DataFlash log binary blocks are large; keep them low-priority so beacon/ACK/COMMAND_ACK stay alive.
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive && !calibrationConfigModeActive()) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_STORAGE_INFORMATION
    case MAVLINK_MSG_ID_STORAGE_INFORMATION:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkHighForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_MOUNT_STATUS
    case MAVLINK_MSG_ID_MOUNT_STATUS:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkHighForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_INFORMATION
    case MAVLINK_MSG_ID_CAMERA_INFORMATION:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkHighForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_SETTINGS
    case MAVLINK_MSG_ID_CAMERA_SETTINGS:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS
    case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED
    case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_FOV_STATUS
    case MAVLINK_MSG_ID_CAMERA_FOV_STATUS:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION
    case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkHighForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_VIDEO_STREAM_STATUS
    case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_ADSB_VEHICLE
    case MAVLINK_MSG_ID_ADSB_VEHICLE:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkHighForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
#ifdef MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
      if (currentSF <= MP_EXTENDED_SETUP_FEATURES_MAX_SF && !paramSyncActive) enqueueMavlinkLowForGCS(msg);
      break;
#endif
    default: break;
  }
}

void readPixhawkMavlink() {
  while (PixhawkSerial.available()) {
    uint8_t c = PixhawkSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavMsg, &mavStatus)) {
      updateTelemetryFromPixhawk(mavMsg);
      enqueuePixhawkMavlinkIfNeeded(mavMsg);
    }
  }
}

bool waitResponseFromGCS(unsigned long timeoutMs, uint32_t expectedCounter, bool requireCounterMatch) {
  uint8_t rxBuf[LORA_RX_MAX]; memset(rxBuf, 0, sizeof(rxBuf));
  radio.standby(); delay(TURNAROUND_MS);
  int16_t state = radio.receive(rxBuf, sizeof(rxBuf), timeoutMs);
  size_t rxLen = radio.getPacketLength(); radio.standby();
  if (state != RADIOLIB_ERR_NONE) return false;
  if (rxLen == 0 || rxLen > LORA_RX_MAX) return false;
  if (!validatePacket(rxBuf, rxLen)) return false;

  // Calculated link quality based on received GCS packets.
  lastGcsDownlinkRssi = radio.getRSSI();
  lastGcsDownlinkSnr = radio.getSNR();
  lastGcsDownlinkMetricMs = millis();

  PacketHeader *hdr = (PacketHeader *)rxBuf;
  if (hdr->type == PKT_LINK_ACK) {
    if (rxLen != sizeof(LinkAckPacket)) return false;
    LinkAckPacket ack; memcpy(&ack, rxBuf, sizeof(ack));
    if (!requireCounterMatch) { lastGcsContactMs = millis(); return true; }
    if (ack.counter == expectedCounter) { lastGcsContactMs = millis(); return true; }
    return false;
  }
  if (hdr->type == PKT_CMD_COMPACT) {
    bool ok = handleCompactCommandFromGCS(rxBuf, rxLen);
    if (ok) lastGcsContactMs = millis();
    return ok;
  }
  if (hdr->type == PKT_MAVLINK_RAW) {
    if (rxLen < RAW_PKT_HEADER_LEN || rxLen > sizeof(MavlinkRawPacket)) return false;
    MavlinkRawPacket raw; memset(&raw, 0, sizeof(raw)); memcpy(&raw, rxBuf, rxLen);
    if (raw.len == 0 || raw.len > RAW_MAVLINK_MAX) return false;
    if (rxLen < (size_t)(RAW_PKT_HEADER_LEN + raw.len)) return false;
    lastGcsContactMs = millis();
    suppressCurrentRawToPixhawk = false;
    inspectRawCommandFromGCS(raw.payload, raw.len);
    if (!suppressCurrentRawToPixhawk) {
      PixhawkSerial.write(raw.payload, raw.len);
    }
    suppressCurrentRawToPixhawk = false;
    return true;
  }
  if (hdr->type == PKT_CONFIG_ACK) {
    if (rxLen != sizeof(ConfigAckPacket)) return false;
    ConfigAckPacket ack; memcpy(&ack, rxBuf, sizeof(ack));
    if (ack.accepted && configPending && ack.next_sf == pendingSF && ack.next_tp == pendingTP && ack.next_profile == pendingProfile && ack.apply_counter == pendingApplyCounter) {
      lastGcsContactMs = millis();
      scheduleConfigApply(ack.next_sf, ack.next_tp, ack.next_profile, ack.apply_counter);
      return true;
    }
    return false;
  }
  return false;
}

bool sendConfigProposal() {
  ConfigProposalPacket pkt = {};
  initHeader(pkt.hdr, PKT_CONFIG_PROPOSE);
  pkt.counter = counter; pkt.apply_counter = pendingApplyCounter;
  pkt.current_sf = currentSF; pkt.current_tp = currentTP;
  pkt.next_sf = pendingSF; pkt.next_tp = pendingTP; pkt.next_profile = pendingProfile;
  finalizePacketCrc(&pkt, sizeof(pkt));
  radio.standby(); radio.setBandwidth(LORA_BW_KHZ); radio.setSpreadingFactor(currentSF); radio.setOutputPower(currentTP);
  int16_t txState = radioTransmitBlocking((const uint8_t *)&pkt, sizeof(pkt));
  if (txState != RADIOLIB_ERR_NONE) { updateMSADR(false); return false; }
  bool ackOK = waitResponseFromGCS(ackTimeoutForSF(currentSF), pkt.counter, false);
  updateMSADR(ackOK); lastAnyTx = millis(); lastConfigTryMs = lastAnyTx;
  return ackOK;
}

bool sendParamBulkPacketToGCS() {
  ParamBulkPacket pkt;
  if (!peekParamBulkPacket(pkt)) return false;
  if (pkt.count == 0 || pkt.count > PARAM_BULK_MAX_RECORDS) { popParamBulkPacket(); return false; }
  uint16_t packetLen = PARAM_BULK_LEN(pkt.count);
  finalizePacketCrc(&pkt, packetLen);
  radio.standby();
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(currentSF);
  radio.setOutputPower(currentTP);
  int16_t txState = radioTransmitBlocking((const uint8_t *)&pkt, packetLen);
  rawTxCount++;
  paramBulkTxCount++;
  if (txState != RADIOLIB_ERR_NONE) {
    paramBulkFailCount++;
    updateMSADR(false);
    return false;
  }
  unsigned long t0 = millis();
  bool ackOK = waitResponseFromGCS(paramBulkAckTimeoutForSF(currentSF), counter, false);
  unsigned long t1 = millis();
  if (ackOK) {
    popParamBulkPacket();
    paramBulkAckCount++;
  } else {
    paramBulkFailCount++;
  }
  if (ackOK && t1 >= t0) lastLatency = (t1 - t0) / 2.0f; else lastLatency = 0.0f;
  updateMSADR(ackOK);
  lastAnyTx = millis();
  return ackOK;
}

bool sendRawPacketToGCS(bool highPriority) {
  MavlinkRawPacket raw; bool ok = highPriority ? peekHighRawPacket(raw) : peekLowRawPacket(raw);
  if (!ok) return false;
  uint16_t packetLen = RAW_PKT_HEADER_LEN + raw.len;
  finalizePacketCrc(&raw, packetLen);
  radio.standby(); radio.setBandwidth(LORA_BW_KHZ); radio.setSpreadingFactor(currentSF); radio.setOutputPower(currentTP);
  int16_t txState = radioTransmitBlocking((const uint8_t *)&raw, packetLen);
  rawTxCount++;
  if (txState != RADIOLIB_ERR_NONE) { updateMSADR(false); return false; }
  unsigned long t0 = millis();
  bool ackOK = waitResponseFromGCS(ackTimeoutForSF(currentSF), counter, false);
  unsigned long t1 = millis();
  if (ackOK) { if (highPriority) popHighRawPacket(); else popLowRawPacket(); }
  if (ackOK && t1 >= t0) lastLatency = (t1 - t0) / 2.0f; else lastLatency = 0.0f;
  updateMSADR(ackOK); lastAnyTx = millis();
  return ackOK;
}



void applyConfig(uint8_t sf, uint8_t tp, uint8_t profile) {
  if (sf < SF_MIN) sf = SF_MIN; if (sf > SF_MAX) sf = SF_MAX;
  if (tp < TP_MIN) tp = TP_MIN; if (tp > TP_MAX) tp = TP_MAX;
  previousSF = currentSF; currentSF = sf; currentTP = tp; currentProfile = profile;
  EEPROM.write(EEPROM_SF_ADDR, currentSF);
  EEPROM.commit();
  applyRadioSettings(currentSF, currentTP);
  configPending = false; scheduledConfig = false; configPendingSinceMs = 0; scheduledConfigSinceMs = 0; lastConfigTryMs = 0;
}

void proposeConfig(uint8_t nextSF, uint8_t nextTP) {
  if (configPending || scheduledConfig) return;
  if (nextSF < SF_MIN) nextSF = SF_MIN; if (nextSF > SF_MAX) nextSF = SF_MAX;
  if (nextTP < TP_MIN) nextTP = TP_MIN; if (nextTP > TP_MAX) nextTP = TP_MAX;
  if (nextSF == currentSF && nextTP == currentTP) return;
  pendingSF = nextSF; pendingTP = nextTP; pendingProfile = profileForSF(nextSF);
  pendingApplyCounter = counter + 1; configPending = true; configPendingSinceMs = millis(); lastConfigTryMs = 0;
}

void scheduleConfigApply(uint8_t sf, uint8_t tp, uint8_t profile, uint32_t applyCounter) {
  if (sf < SF_MIN) sf = SF_MIN; if (sf > SF_MAX) sf = SF_MAX;
  if (tp < TP_MIN) tp = TP_MIN; if (tp > TP_MAX) tp = TP_MAX;
  scheduledSF = sf; scheduledTP = tp; scheduledProfile = profile;
  scheduledApplyCounter = applyCounter; scheduledConfig = true; scheduledConfigSinceMs = millis();
  configPending = false; configPendingSinceMs = 0; lastConfigTryMs = 0;
}

void applyScheduledConfigAfterAckedTelemetry(uint32_t ackedCounter) {
  if (!scheduledConfig) return;
  if (scheduledApplyCounter != 0 && ackedCounter < scheduledApplyCounter) return;
  applyConfig(scheduledSF, scheduledTP, scheduledProfile);
}


bool sendParamDebugPacketToGCS() {
#if PARAM_DEBUG_ENABLE
  if (!paramSyncActive) return false;
  unsigned long now = millis();
  if (now - lastParamDebugTxMs < PARAM_DEBUG_MIN_INTERVAL_MS) return false;

  ParamDebugPacket pkt = {};
  initHeader(pkt.hdr, PKT_PARAM_DEBUG);
  pkt.dbg_async_start = (uint16_t)asyncParamPollStartCount;
  pkt.dbg_async_req_tx = (uint16_t)asyncParamRequestTxCount;
  pkt.dbg_async_retry = (uint16_t)asyncParamRequestRetryCount;
  pkt.dbg_param_bulk_queued = (uint16_t)paramBulkQueuedCount;
  pkt.dbg_param_bulk_tx = (uint16_t)paramBulkTxCount;
  pkt.dbg_param_bulk_ack = (uint16_t)paramBulkAckCount;
  pkt.dbg_param_bulk_fail = (uint16_t)paramBulkFailCount;
  pkt.dbg_param_value_drop = (uint16_t)paramValueDrop;
  pkt.dbg_full_param_blocked = (uint16_t)fullParamSyncBlockedHighSfCount;
  pkt.dbg_async_index = currentAsyncParamIndex;
  pkt.dbg_async_total = asyncParamTotalCount;
  pkt.dbg_param_state = (paramSyncActive ? 1 : 0) |
                        (isAsyncParamSyncActive ? 2 : 0) |
                        (asyncParamWaitingForValue ? 4 : 0);
  finalizePacketCrc(&pkt, sizeof(pkt));

  radio.standby();
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(currentSF);
  radio.setOutputPower(currentTP);
  int16_t txState = radioTransmitBlocking((const uint8_t *)&pkt, sizeof(pkt));
  if (txState == RADIOLIB_ERR_NONE) {
    rawTxCount++;
    lastParamDebugTxMs = now;
    lastAnyTx = millis();
    return true;
  }
#endif
  return false;
}

bool sendTelemetryPacketToGCS() {
  uint8_t profile = selectTelemetryProfile();
  currentProfile = profile;
  if (calibrationConfigModeActive()) linkMode = LINK_MODE_CALIBRATION;
  else if (paramSyncActive) linkMode = LINK_MODE_PARAM_SYNC;
  else linkMode = LINK_MODE_NORMAL;

  unsigned long txStartMs = millis();
  radio.standby();
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(currentSF);
  radio.setOutputPower(currentTP);

  TelemetryBeaconPacket pkt = {};
  initHeader(pkt.hdr, PKT_TELEM_BEACON);
  pkt.counter = counter++;
  pkt.radio_packed = packRadioParams78(currentSF, currentTP, profile);
#if UAV_FORCE_REMOTE_RSSI_FOR_MP_UI
  // User-requested fixed remote RSSI for MP/GCS display stability.
  // SNR still uses the real measurement when available; RSSI is exported as
  // -17 dBm equivalent. This is display-only and does not affect modem RF.
  if (lastGcsDownlinkMetricMs != 0 && (millis() - lastGcsDownlinkMetricMs) < 15000UL) {
    pkt.remote_snr_x2 = loraSnrDbToX2(lastGcsDownlinkSnr);
  } else {
    pkt.remote_snr_x2 = -128;
  }
  pkt.remote_rssi_q = loraRssiDbmToQ254(UAV_FORCED_REMOTE_RSSI_DBM);
#else
  if (lastGcsDownlinkMetricMs != 0 && (millis() - lastGcsDownlinkMetricMs) < 15000UL) {
    pkt.remote_snr_x2 = loraSnrDbToX2(lastGcsDownlinkSnr);
    pkt.remote_rssi_q = loraRssiDbmToQ254(lastGcsDownlinkRssi);
  } else {
    pkt.remote_snr_x2 = -128;  // sentinel: no real downlink metric yet
    pkt.remote_rssi_q = 0;
  }
#endif
  pkt.link_mode = linkMode;
  pkt.latency_x100 = (uint16_t)(lastLatency * 100.0f + 0.5f);
  pkt.meta = makeTelemetryMeta();
  fillBeacon(pkt.data);
  finalizePacketCrc(&pkt, sizeof(pkt));

  int16_t txState = radioTransmitBlocking((const uint8_t *)&pkt, sizeof(pkt));
  telemetryTxCount++;
  if (txState != RADIOLIB_ERR_NONE) { updateMSADR(false); return false; }

  float toaMs = estimateLoRaToA_ms(currentSF, LORA_BW_KHZ * 1000.0f, LORA_CR_DEN, sizeof(pkt));
  float energyMJ = estimatePacketEnergy_mJ(currentTP, toaMs);
  telemetryTxAttemptsMetric++;
  telemetryTxEnergyMetric_mJ += energyMJ;
  telemetryTxToAMetric_ms += toaMs;

  bool expectAck = shouldExpectTelemetryAck(pkt.counter, currentSF);
  bool listenForDownlink = expectAck || shouldListenForCommandDownlink(currentSF);
  bool ackOK = true;
  if (listenForDownlink) {
    unsigned long t0 = millis();
    unsigned long rxWindow = expectAck ? telemetryAckTimeoutForSF(currentSF) : commandDownlinkTimeoutForSF(currentSF);
    ackOK = waitResponseFromGCS(rxWindow, pkt.counter, expectAck);
    unsigned long t1 = millis();
    if (expectAck) {
      if (ackOK && t1 >= t0) lastLatency = (t1 - t0) / 2.0f;
      else lastLatency = 0.0f;
      if (ackOK) applyScheduledConfigAfterAckedTelemetry(pkt.counter);

      // During parameter sync, PARAM_BULK ACK is the authoritative link-health
      // signal. Do not let a decimated telemetry ACK miss force MSADR fallback
      // or inject long recovery behavior while bulk transfer is healthy.
      if (!paramSyncActive || ackOK) updateMSADR(ackOK);
    } else {
      // Opportunistic command slot. Timeout here is not a link failure; it only means no downlink command was waiting.
      if (ackOK) { lastLatency = (t1 >= t0) ? ((t1 - t0) / 2.0f) : 0.0f; flightCommandDownlinkSlotCount++; }
    }
  }

  // lastTelemetryTx tracks packet start times to keep the beacon interval near 1.0 seconds.
  lastTelemetryTx = txStartMs;
  lastAnyTx = millis();
  return expectAck ? ackOK : true;
}

bool sendFallbackBeaconIfNeeded() {
  if (consecutiveLinkFails < FALLBACK_FAIL_COUNT) return false;
  unsigned long now = millis();
  if (now - lastFallbackBeaconMs < FALLBACK_BEACON_INTERVAL_MS) return false;
  lastFallbackBeaconMs = now; fallbackBeaconCount++;
  return sendTelemetryPacketToGCS();
}

void enforceHighSfNoParamSync() {
  // High spreading factors enforce regular telemetry beacons and compact payloads.
  // but must not carry full param-sync/bulk setup traffic.
  if (linkSfSupportsSetup(currentSF)) return;
  if (paramSyncActive) { abortParamSyncRecovery("high_sf_no_param_sync"); return; }
  if (paramBulkCount > 0) flushParamBulkQueue();
}

void setup() {
  Serial.begin(115200);
  delay(BOOT_STABILIZE_MS);
  bootMs = millis();

  // EEPROM initialization. In UAV-master PHY mode, the effective SF is determined on the UAV.
  // GCS does not need to be re-uploaded since it will scan and lock onto the UAV beacon.
  EEPROM.begin(512);
#if PHY_TEST_FORCE_DEFAULT_SF
  currentSF = DEFAULT_SF;
  EEPROM.write(EEPROM_SF_ADDR, currentSF);
  EEPROM.commit();
#else
  uint8_t savedSF = EEPROM.read(EEPROM_SF_ADDR);
  if (savedSF < SF_MIN || savedSF > SF_MAX) {
    savedSF = DEFAULT_SF;
    EEPROM.write(EEPROM_SF_ADDR, savedSF);
    EEPROM.commit();
  }
  currentSF = savedSF;
#endif
  currentTP = DEFAULT_TP;
  currentProfile = PROFILE_BEACON;

  Serial.print("[BOOT UAV] DEFAULT_SF=");
  Serial.print(DEFAULT_SF);
  Serial.print(" DEFAULT_TP=");
  Serial.print(DEFAULT_TP);
  Serial.print(" currentSF=");
  Serial.print(currentSF);
  Serial.print(" currentTP=");
  Serial.print(currentTP);
  Serial.print(" masterPHY=");
  Serial.println(UAV_MASTER_PHY_MODE);

  initPixhawkSerialPort();
  spi.begin(18, 19, 23, LORA_SS);
  if (!initLoRaRadioUAV()) {
    Serial.println("[BOOT UAV] LoRa init failed after retries, restarting ESP32");
    delay(LORA_BOOT_FAIL_RESTART_DELAY_MS);
    ESP.restart();
  }
  autopilotTargetSysId = 1;
  autopilotTargetCompId = MAV_COMP_ID_AUTOPILOT1;
  autopilotIdentityLocked = false;
  pixDataFull.system_id = autopilotTargetSysId; pixDataFull.component_id = autopilotTargetCompId;
  pixDataFull.mav_type = MAV_TYPE_HEXAROTOR; pixDataFull.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  pixDataFull.system_status = MAV_STATE_STANDBY;
  resetBatteryTelemetryDefaults();
  configurePixhawkMessageIntervalsNormal();
  lastStreamConfigMs = millis(); streamConfigRetry = 1; streamConfigRetryCount = 1;
  lastTelemetryTx = millis(); lastLowRawTx = millis(); lastParamBulkTx = millis(); lastAnyTx = millis();
  lastConfigTryMs = millis(); lastAckOkMs = millis(); lastGcsContactMs = millis(); lastRadioRecoveryMs = millis();
  resetLinkQualityAfterParamSync();
}

void loop() {
  readPixhawkMavlink();
  recoverPixhawkSerialIfSilent();
  updateParamWriteAckState();
  updatePixhawkStreamConfig();
  updateParamSyncTimeout();
  updateAsyncParamPolling();
  updateFlightCommandPriorityMode();
  if (!paramSyncActive && !calibrationConfigModeActive() && paramSyncStreamSlowed) { exitParamSyncStreamMode(); linkMode = LINK_MODE_NORMAL; }
  updateCalibrationConfigMode();
  enforceHighSfNoParamSync();
  updateConfigStuckGuards();
  updateRadioSoftRecovery();
  sendSyntheticGCSHeartbeatToPixhawkIfNeeded();
  // ARM/DISARM from Mission Planner is forwarded as raw MAVLink.
  // Auto-retry is not called so ArduPilot's pre-arm safety is not triggered repeatedly.

  unsigned long now = millis();
  if (now - lastAnyTx < MIN_SEND_GAP_MS) return;
  if (sendFallbackBeaconIfNeeded()) return;
  if (configPending && now - lastConfigTryMs >= CONFIG_RETRY_MS) { sendConfigProposal(); return; }

  bool telemetryDue = (now - lastTelemetryTx >= intervalForSF(currentSF));
  // SF12 needs a strict 1 Hz beacon because Mission Planner heartbeat comes from
  // reconstructed real FC state in the beacon. Do not let queued COMMAND_ACK/STATUSTEXT
  // starve the beacon at SF12; the ACK/STATUSTEXT will be sent immediately after.
  if (currentSF >= 12 && telemetryDue) { sendTelemetryPacketToGCS(); return; }

  // Hard command responses remain above PARAM_BULK: safety-critical COMMAND_ACK
  // / STATUSTEXT must not be delayed by a large parameter transfer.
  if (commandModeActive() && highCount > 0) { sendRawPacketToGCS(true); return; }

  /*
   * PARAM_BULK priority:
   * During full sync, telemetry is useful but PARAM_BULK is the payload that
   * advances Mission Planner's x/y progress. Put bulk before normal telemetry
   * so a ready bulk cannot be starved by the beacon cadence.
   */
  if (paramSyncActive && paramBulkCount > 0) {
    ParamBulkPacket pkPeek;
    if (peekParamBulkPacket(pkPeek)) {
      uint8_t limit = paramBulkRecordsLimitForCurrentSF();
      bool full = (pkPeek.count >= limit);
      bool hasNewer = (paramBulkCount > 1);
      bool asyncComplete = (!isAsyncParamSyncActive && asyncParamTotalCount > 0 &&
                            currentAsyncParamIndex >= asyncParamTotalCount);
      bool idle = (lastParamValueMs > 0 && now - lastParamValueMs >= paramBulkFillGraceForSF(currentSF));
      /*
       * Tail-completion fast path:
       * When the async poller already accepted the last known parameter, do not
       * hold the final partial PARAM_BULK for the normal fill-grace timer. This
       * removes the common end-of-sync "waiting at the last few parameters"
       * pause while keeping normal mid-sync batching intact.
       */
      if ((full || hasNewer || idle || asyncComplete) && now - lastParamBulkTx >= PARAM_BULK_TX_INTERVAL_MS) {
        if (sendParamBulkPacketToGCS()) lastParamBulkTx = millis();
        return;
      }
    }
  }

  // Debug is best-effort and rate-limited to 20 s. It is sent only after
  // PARAM_BULK had priority, so it cannot stall the progress bar.
  if (sendParamDebugPacketToGCS()) return;

  // Calibration and command feedback must reach Mission Planner before regular
  // beacons, otherwise accel/compass progress can appear frozen on slow links.
  if (highCount > 0) { sendRawPacketToGCS(true); return; }
#ifdef MAVLINK_MSG_ID_MAG_CAL_PROGRESS
  // MAG_CAL_PROGRESS is prioritized with a latest-value cache per compass_id and bundling.
  if (sendPendingMagCalProgressToGCS()) return;
#endif

  // Keep beacon alive, but only after control/setup feedback had a chance.
  if (telemetryDue) { sendTelemetryPacketToGCS(); return; }

  if (scheduledConfig) { sendTelemetryPacketToGCS(); return; }
  if (activeConfigOrParamMode() && lowCount > 0 && now - lastLowRawTx >= lowRawIntervalForSF(currentSF)) {
    if (sendRawPacketToGCS(false)) lastLowRawTx = millis();
    return;
  }
  if (!activeConfigOrParamMode() && lowCount > 0 && currentSF <= 9 && now - lastLowRawTx >= lowRawIntervalForSF(currentSF)) {
    if (sendRawPacketToGCS(false)) lastLowRawTx = millis();
    return;
  }
}
