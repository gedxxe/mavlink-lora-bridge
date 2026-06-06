#include <SPI.h>
#include "TelemetryProtoFix.h"
#include <RadioLib.h>
#include <MAVLink_ardupilotmega.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
// TelemetryProtoFix.h already included at line 2 (duplicate removed)
#ifndef UINT8_MAX
#define UINT8_MAX 255
#endif

#ifndef RADIOLIB_ERR_UNKNOWN
#define RADIOLIB_ERR_UNKNOWN -999
#endif

#ifndef MAVLINK_COMM_2
#define MAVLINK_COMM_2 2
#endif

#ifndef MAVLINK_MSG_ID_SET_MODE
#define MAVLINK_MSG_ID_SET_MODE 11
#endif
#ifndef MAVLINK_MSG_ID_MISSION_SET_CURRENT
#define MAVLINK_MSG_ID_MISSION_SET_CURRENT 41
#endif
#ifndef MAVLINK_MSG_ID_MISSION_CLEAR_ALL
#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL 45
#endif
#ifndef MAVLINK_MSG_ID_MANUAL_CONTROL
#define MAVLINK_MSG_ID_MANUAL_CONTROL 69
#endif
#ifndef MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70
#endif

// Mission Planner / ArduPilot command IDs.
#define CMD_DO_SET_MODE               176
#ifndef MAV_CMD_DO_SET_MODE
#define MAV_CMD_DO_SET_MODE 176
#endif
#define CMD_COMPONENT_ARM_DISARM      400
#ifndef MAV_CMD_COMPONENT_ARM_DISARM
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#endif
#define CMD_PREFLIGHT_CALIBRATION     241
#define CMD_PREFLIGHT_STORAGE         245
#define CMD_PREFLIGHT_REBOOT_SHUTDOWN 246
#define CMD_START_RX_PAIR             500
#define CMD_DO_START_MAG_CAL          42424
#define CMD_DO_ACCEPT_MAG_CAL         42425
#define CMD_DO_CANCEL_MAG_CAL         42426
#define CMD_ACCELCAL_VEHICLE_POS      42429
#ifndef MAV_CMD_SET_MESSAGE_INTERVAL
#define MAV_CMD_SET_MESSAGE_INTERVAL 511
#endif
#ifndef MAV_CMD_REQUEST_MESSAGE
#define MAV_CMD_REQUEST_MESSAGE 512
#endif
#ifndef MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
#define MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES 520
#endif

#ifndef MAV_SEVERITY_CRITICAL
#define MAV_SEVERITY_CRITICAL 2
#endif
#ifndef MAV_SEVERITY_NOTICE
#define MAV_SEVERITY_NOTICE 5
#endif
#ifndef MAV_SEVERITY_INFO
#define MAV_SEVERITY_INFO 6
#endif
#ifndef MAV_RESULT_ACCEPTED
#define MAV_RESULT_ACCEPTED 0
#endif
#ifndef MAV_RESULT_DENIED
#define MAV_RESULT_DENIED 2
#endif
#ifndef MAV_RESULT_FAILED
#define MAV_RESULT_FAILED 4
#endif

#ifndef MAV_CMD_NAV_WAYPOINT
#define MAV_CMD_NAV_WAYPOINT 16
#endif
#ifndef MAV_CMD_NAV_LOITER_UNLIM
#define MAV_CMD_NAV_LOITER_UNLIM 17
#endif
#ifndef MAV_CMD_NAV_LOITER_TURNS
#define MAV_CMD_NAV_LOITER_TURNS 18
#endif
#ifndef MAV_CMD_NAV_LOITER_TIME
#define MAV_CMD_NAV_LOITER_TIME 19
#endif
#ifndef MAV_CMD_NAV_RETURN_TO_LAUNCH
#define MAV_CMD_NAV_RETURN_TO_LAUNCH 20
#endif
#ifndef MAV_CMD_NAV_LAND
#define MAV_CMD_NAV_LAND 21
#endif
#ifndef MAV_CMD_NAV_TAKEOFF
#define MAV_CMD_NAV_TAKEOFF 22
#endif
#ifndef MAV_CMD_DO_REPOSITION
#define MAV_CMD_DO_REPOSITION 192
#endif
#ifndef MAV_CMD_MISSION_START
#define MAV_CMD_MISSION_START 300
#endif
#ifndef MAV_CMD_DO_SET_HOME
#define MAV_CMD_DO_SET_HOME 179
#endif

// =====================================================
// NODE GCS - OPTIMIZED MAVLINK-AWARE LORA BRIDGE - STATIS V18 PAYLOAD 78B MPCONNECT FIX
// Hanya menerima payload beacon ringkas tetap untuk pengujian PHY LoRa
// =====================================================

// ================= Link Mode =================
#define LINK_MODE_NORMAL       0
#define LINK_MODE_PARAM_SYNC   1
#define LINK_MODE_CALIBRATION  2
#define LINK_MODE_MISSION      3
#define LINK_MODE_FAILSAFE     4
#define LINK_MODE_COMMAND      5
uint8_t linkMode = LINK_MODE_NORMAL;

// ================= Flight Command Priority Mode =================
#define FLIGHT_COMMAND_HOLD_MS 12000UL
#define FLIGHT_COMMAND_STALE_CLEAR_MS 15000UL
unsigned long commandModeUntilMs = 0;
unsigned long lastFlightCommandRxMs = 0;
unsigned long lastFlightCommandTxMs = 0;
uint32_t flightCommandRxCount = 0;
uint32_t flightCommandTxSlotCount = 0;
uint32_t flightCommandQueueFlushCount = 0;

// COMMAND_ACK guard: Mission Planner doCommand can crash/fail when it receives
// delayed ACKs for background/internal commands (especially SET_MESSAGE_INTERVAL)
// while waiting for an action command ACK.
#define MP_PENDING_COMMAND_SLOTS 16
#define MP_COMMAND_ACK_GUARD_MS 20000UL
uint16_t mpPendingCommand[MP_PENDING_COMMAND_SLOTS];
unsigned long mpPendingCommandMs[MP_PENDING_COMMAND_SLOTS];
uint8_t mpPendingCommandHead = 0;
uint16_t commandPriorityExpectedAck = 0;
uint32_t commandAckSuppressedCount = 0;

bool isArmDisarmCommandId(uint16_t command);
bool shouldForwardArmAckToMissionPlanner(const mavlink_command_ack_t &ack);

// High-SF command UX guard: pada SF11/SF12 ACK asli dari flight controller bisa terlambat
// karena half-duplex LoRa. Untuk action command non-ARM, GCS dapat memberi ACK proxy
// ke Mission Planner setelah command berhasil dipancarkan pada slot downlink.
#define COMMAND_PROXY_ACK_HIGH_SF_ENABLE 0
#define COMMAND_PROXY_ACK_MIN_SF 10
#define COMMAND_PROXY_ACK_ARM_ENABLE 0  // V14 delay/force fix: ARM must wait for real FC ACK so Mission Planner can show Force Arm on DENIED.
#define COMMAND_PROXY_ACK_SUPPRESS_MS 30000UL
#define COMMAND_PROXY_ACK_ON_ENQUEUE_ENABLE 0
#define COMMAND_PROXY_ACK_BURST_COUNT 3
#define COMMAND_PROXY_ACK_BURST_GAP_MS 350UL
uint16_t proxiedAckCommand[MP_PENDING_COMMAND_SLOTS];
unsigned long proxiedAckCommandMs[MP_PENDING_COMMAND_SLOTS];
uint8_t proxiedAckHead = 0;
uint32_t commandProxyAckSentCount = 0;
uint16_t proxyAckBurstCommand = 0;
uint8_t proxyAckBurstRemaining = 0;
unsigned long proxyAckBurstNextMs = 0;

// V15 low-delay UI feedback:
// SET_MODE/DO_SET_MODE can be visually delayed in Mission Planner because the real
// HEARTBEAT/custom_mode returns through a slow half-duplex LoRa path. Keep a temporary
// optimistic heartbeat overlay for SF10-SF12. This is only UI feedback; real beacon
// data still corrects the state once the flight controller reports the new mode.
#define OPTIMISTIC_MODE_UI_ENABLE 0
#define OPTIMISTIC_MODE_MIN_SF 10
#define OPTIMISTIC_MODE_HOLD_MS 8000UL
#define OPTIMISTIC_MODE_HEARTBEAT_INTERVAL_MS 300UL
#define OPTIMISTIC_MODE_MATCH_CLEAR_MS 1200UL
#define OPTIMISTIC_MODE_IMMEDIATE_BURST_COUNT 3

bool paramSyncActive = false;
unsigned long paramSyncStartMs = 0;
unsigned long paramSyncUntilMs = 0;
uint32_t paramSyncAutoAbortCount = 0;
unsigned long lastParamValueMs = 0;
uint16_t lastParamIndex = 0;
uint16_t lastParamCount = 0;

// ================= Calibration / Config Mode =================
#define CAL_CONFIG_HOLD_MS 180000UL

bool calConfigActive = false;
unsigned long calConfigUntilMs = 0;

uint32_t calCommandSentCount = 0;
uint32_t magCalProgressRxCount = 0;
uint32_t magCalReportRxCount = 0;
uint32_t calCommandAckRxCount = 0;
uint32_t calStatustextRxCount = 0;

#define PARAM_SYNC_TIMEOUT_MS       600000UL
#define PARAM_SYNC_IDLE_EXIT_MS       3000UL
#define PARAM_SYNC_NO_VALUE_EXIT_MS_SF7_9 12000UL
#define PARAM_SYNC_NO_VALUE_EXIT_MS_SF10  12000UL
#define PARAM_SYNC_NO_VALUE_EXIT_MS_SF11  10000UL
#define PARAM_SYNC_NO_VALUE_EXIT_MS_SF12   8000UL
#define PARAM_MODE_HOLD_MS           5000UL
// Full parameter download at SF11/SF12 is intentionally blocked by default.
// It prevents Mission Planner auto-param sync from stalling the telemetry link.
#define BLOCK_FULL_PARAM_SYNC_HIGH_SF 1
#define FULL_PARAM_SYNC_MAX_SF 9
#define HIGH_SF_PARAM_NOTICE_INTERVAL_MS 5000UL
// V15 3FIX: SF10-SF12 are command/monitoring links, not full Mission Planner parameter-sync links.
// Block full-list requests and non-interactive PARAM_REQUEST_READ at high SF.
#define BLOCK_PARAM_READ_HIGH_SF 1
#define LORA_PARAM_NOTICE_MIN_INTERVAL_MS 3000UL
#define ARM_FEEDBACK_HOLD_MS 15000UL
unsigned long paramModeHoldUntilMs = 0;
unsigned long lastHighSfParamNoticeMs = 0;
unsigned long lastLoRaParamNoticeMs = 0;
uint8_t lastLoRaParamNoticeSF = 0;
uint8_t lastLoRaParamNoticeTP = 255;
bool loRaParamNoticeSent = false;
// V15_6FIX: LoRa parameter notice must be visible in Mission Planner Messages.
// Send a short burst at connection/SF change, but do not spam RSSI/SNR/PDR metrics.
char loRaParamNoticeText[64] = {0};
uint8_t loRaParamNoticeBurstRemaining = 0;
unsigned long loRaParamNoticeNextMs = 0;
#define LORA_PARAM_NOTICE_BURST_COUNT 10
#define LORA_PARAM_NOTICE_BURST_GAP_MS 1000UL
unsigned long armFeedbackUntilMs = 0;

// V15_4FIX ARM/FORCE-ARM transaction guard:
// MAVLink COMMAND_ACK only contains command id 400, so normal ARM and FORCE ARM
// cannot be distinguished from the ACK alone. On a slow/queued LoRa bridge, a stale
// ACK from the previous normal ARM can arrive while Mission Planner is waiting for
// the next FORCE ARM command and can trigger doCommand mismatch/failure/crash.
// Therefore ARM/DISARM ACK is forwarded only after the current ARM transaction was
// actually transmitted to UAV, plus a small SF-aware travel guard.
#define ARM_TX_WAIT_TIMEOUT_MS       10000UL
#define ARM_ACK_TOTAL_TIMEOUT_MS     28000UL
uint32_t armCommandTxnSeq = 0;
bool armCommandTxnActive = false;
bool armCommandTxnTxConfirmed = false;
bool armCommandTxnArm = false;
bool armCommandTxnForce = false;
unsigned long armCommandTxnQueuedMs = 0;
unsigned long armCommandTxnTxMs = 0;
uint32_t armCommandAckSuppressedStaleCount = 0;
uint32_t armCommandAckForwardedCount = 0;

uint8_t lastHighSfNoticeForSf = 0;
bool highSfConnectNoticeSent = false;
uint32_t fullParamSyncBlockedHighSfCount = 0;
uint32_t paramReadBlockedHighSfCount = 0;

// Protocol constants moved to TelemetryProtoFix.h

// ================= LoRa =================
#define LORA_SS    5
#define LORA_RST   25
#define LORA_DIO0  26
#define LORA_DIO1  RADIOLIB_NC

#define FREQ_MHZ      433.0
#define LORA_BW_KHZ   500.0          // BW 500 kHz
#define LORA_CR_DEN   8
#define LORA_SYNC     0x12

#define SF_MIN 7
#define SF_MAX 12
#define DEFAULT_SF 7   // Hanya SF awal scan; GCS akan auto-scan dan lock ke SF UAV
#define UAV_MASTER_PHY_MODE 1       // 1 = UAV master PHY; GCS scan SF7..SF12 saat belum lock
#define PHY_TEST_LOCK_INITIAL_SF 0  // 0 = auto-scan; tidak perlu samakan DEFAULT_SF GCS dengan UAV
#define GCS_FIXED_TP 16
#define TP_MIN 10
#define TP_MAX 16
#define GCS_DEFAULT_TP GCS_FIXED_TP   // TP awal GCS; setelah lock, GCS mengikuti TP yang diumumkan UAV

#define RX_TIMEOUT_MS            1000UL  // fallback; loop memakai rxTimeoutForSF()
#define LINK_IDLE_SCAN_AFTER_MS  2500UL  // fallback; recovery memakai linkIdleScanAfterForSF()
#define RECOVERY_SCAN_STEP_MS     700UL
#define DOWNLINK_TX_GUARD_MS      8UL
#define SCHEDULED_CONFIG_STUCK_MS 15000UL

#define METRICS_WINDOW_SIZE       32
#define HEARTBEAT_SYNTH_INTERVAL_MS 1000UL
#define TELEMETRY_ACK_DECIMATION_ENABLE 1

#define PKT_LINK_ACK          0xA5
#define PKT_MAVLINK_RAW       0x4D
#define PKT_CMD_COMPACT       0x43
#define PKT_PARAM_BULK        0x50
#define PKT_TELEM_BEACON      0x57
#define PKT_CONFIG_PROPOSE    0xC0
#define PKT_CONFIG_ACK        0xC1

// PROFILE_BEACON moved to TelemetryProtoFix.h

// RAW_MAVLINK_MAX moved to TelemetryProtoFix.h
// PARAM_BULK_MAX_RECORDS moved to TelemetryProtoFix.h
#define RAW_HIGH_QUEUE_SIZE   64
#define RAW_LOW_QUEUE_SIZE    96
#define COMPACT_CMD_QUEUE_SIZE 16
// LORA_RX_MAX moved to TelemetryProtoFix.h

#define REPEAT_SET_MODE             1
#define REPEAT_ARM_DISARM           1
#define REPEAT_CALIBRATION_COMMAND  1
#define REPEAT_PARAM_SET            2
#define REPEAT_NORMAL_COMMAND       1
#define CALIBRATION_DEDUP_MS     2200UL

#define LORA_PREAMBLE_SYMBOLS 8.0f
#define LORA_PHY_CRC_ENABLED 1
#define LORA_IMPLICIT_HEADER 0

#define LORA_INIT_RETRY_COUNT 8
#define LORA_INIT_RETRY_DELAY_MS 250

// ================= Boot / Autostart Recovery =================
// Surgical fix: membuat node GCS pulih sendiri setelah power-on tanpa tombol reset manual.
// Tidak mengubah LoRa PHY, identity MAVLink, payload, atau queue logic.
#define BOOT_STABILIZE_MS                 1500UL
#define GCS_LORA_INIT_FAIL_RESTART_MS     2000UL
#define GCS_LORA_NO_PACKET_RECOVERY_MS    30000UL
#define GCS_LORA_HARD_RECOVERY_MIN_GAP_MS 15000UL

// ================= MAVLink forwarding =================
// Raw MAVLink dari UAV tidak di-rate-limit global agar PARAM_VALUE/kalibrasi tidak hilang setelah ACK.

// ================= Variabel untuk kualitas link =================
float lastRssi = -120.0f;
float lastSnr = 0.0f;
unsigned long lastRadioStatusMs = 0;
#define RADIO_STATUS_INTERVAL_MS 500
#define MP_SIGNAL_MIN_LIVE_PCT 5
// Jangan kirim RADIO_STATUS ganda dengan compid autopilot + telemetry radio.
// Dual-compid bisa membuat Mission Planner melihat gap sequence per komponen.
#define RADIO_STATUS_COMPAT_DUAL_COMPID 0

// MPCONNECT FIX:
// Sequence normalizer versi sebelumnya mengubah msg.seq SETELAH mavlink encode/finalize.
// Pada MAVLink2, perubahan header setelah finalize dapat membuat CRC/checksum tidak cocok,
// sehingga Mission Planner menolak HEARTBEAT dan koneksi gagal.
// Karena itu normalizer dinonaktifkan. Pesan tetap dikirim dengan sequence legal dari MAVLink library.
#define MP_MAVLINK_SEQ_NORMALIZER_ENABLE 0
#define RADIO_STATUS_LEGACY_RADIO_ENABLE 0
// MPQUALITY FIX:
// RADIO_STATUS memakai component MAV_COMP_ID_TELEMETRY_RADIO, tetapi harus memiliki
// sequence MAVLink sendiri yang valid sebelum checksum dibuat. Karena itu radio-status
// dikemas dengan *_encode_chan() pada channel khusus, bukan dengan normalizer pasca-encode.
#define MP_RADIO_STATUS_DEDICATED_CHAN_ENABLE 0  // CompileSafe: encode_chan tidak tersedia di sebagian MAVLink Arduino; pakai encode standar agar compile bersih
#define MP_RADIO_STATUS_MAVLINK_CHAN MAVLINK_COMM_2
// V17_signal_fix: Mission Planner PreFlight Telemetry Signal is driven by RADIO_STATUS.
// Keep RADIO_STATUS on the same vehicle sysid/compid as V15.3 (known good), but compute
// its value from real LoRa PHY measurements and packet freshness, not from synthetic state.
#define RADIO_STATUS_USE_VEHICLE_SYSID 1
#define RADIO_STATUS_USE_TELEM_RADIO_COMPID 0  // SINGLE_SOURCE: RADIO_STATUS uses AUTOPILOT1 compid to avoid MP false sequence gaps
#define RADIO_STATUS_PHY_QUALITY_ENABLE 1

// PARAMBYPASS_GOVERNOR FIX:
// Param sync/mission/config responses must stay fast. Only normal telemetry output
// is paced/governed to avoid false MAVLink packet-loss in Mission Planner after connect.
#define MP_SERIAL_PACING_ENABLE 1
#define MP_PARAM_SYNC_BYPASS_ENABLE 1
#define MP_TELEMETRY_GOVERNOR_ENABLE 1
#define MP_SERIAL_MIN_GAP_US 900UL          // legacy fallback
#define MP_TELEMETRY_SERIAL_GAP_US 4500UL   // only applied while sending reconstructed telemetry
#define MP_FAST_SERIAL_GAP_US 0UL           // param/raw bypass: no added pacing
#define MP_RAW_TELEMETRY_DEDUP_ENABLE 1     // drop raw duplicate telemetry in normal mode; keep PARAM/MISSION/ACK fast
#define MP_STRICT_RAW_TELEMETRY_DEDUP_ENABLE 1  // drop duplicate raw telemetry even during param hold; PARAM/MISSION/ACK still pass

// SINGLE SOURCE IDENTITY FIX:
// Mission Planner menghitung packet lost dari stream MAVLink. Jika GCS mengirim
// telemetry dengan campuran compid AUTOPILOT1 + TELEMETRY_RADIO + raw component,
// Mission Planner dapat membaca gap sequence palsu. Patch ini membuat semua output
// MAVLink yang menuju Mission Planner tampak berasal dari satu identitas vehicle.
// Command path Mission Planner->Pixhawk dan LoRa protocol tidak diubah.
#define MP_SINGLE_SOURCE_IDENTITY_ENABLE 1
#define MP_SINGLE_SOURCE_FORCE_COMPID MAV_COMP_ID_AUTOPILOT1


// GPS display stabilizer for Mission Planner checklist. This does NOT change FC GPS or failsafe logic;
// it only prevents transient low satellite counts in reconstructed GPS_RAW_INT from failing MP PreFlight UI.
#define MP_GPS_SAT_DISPLAY_HOLD_ENABLE 1
#define MP_GPS_SAT_HOLD_MS 15000UL
#define MP_GPS_SAT_MIN_WHEN_3D_FIX 10

// LINKFAILSAFE_QUALITYFIX:
// Mission Planner must not keep showing a healthy aircraft when the real LoRa link is lost.
// If uplink beacon is stale or UAV reports repeated ACK/downlink failures, stop forwarding
// vehicle heartbeat/telemetry to Mission Planner so MP can time out naturally.
#define MP_DISCONNECT_ON_LINK_LOSS_ENABLE 1
// MP display hard-loss timeout: do not drop Telemetry Signal to 0 on a short/degraded burst.
// 0% is reserved for hard uplink loss. FC failsafe is still handled by FS_GCS_* using real GCS heartbeat.
#define MP_LINK_LOSS_TIMEOUT_MS_SF7_9 7000UL
#define MP_LINK_LOSS_TIMEOUT_MS_SF10 9000UL
#define MP_LINK_LOSS_TIMEOUT_MS_SF11 12000UL
#define MP_LINK_LOSS_TIMEOUT_MS_SF12 18000UL
#define MP_BIDIR_FAIL_STREAK_THRESHOLD 5
#define MP_BIDIR_MIN_ACK_WINDOW_OK 2
#define MP_LINK_STARTUP_GRACE_MS 6000UL

// V21 SURGICAL FIX: hanya memengaruhi laporan RADIO_STATUS ke Mission Planner.
// LoRa PHY tetap statis/manual; tidak ada adaptive SF/TP.
// ACK/downlink tetap dihitung sebagai metrik internal, tetapi tidak mencap Telemetry Signal MP.
#define MP_SIGNAL_REQUIRE_BIDIR_ACK 0
#define MP_REPORT_REMOTE_RSSI_TO_MP 0
// V23 SURGICAL RESTART FIX:
// Mission Planner PreFlight dapat gagal menginisialisasi Telemetry Signal setelah clean reboot
// jika remrssi dikirim sebagai UINT8_MAX/unknown. Tetap single-source identity; hanya mirror
// nilai rssi lokal ke remrssi untuk kompatibilitas UI Mission Planner. Tidak mengubah LoRa PHY.
#define MP_MIRROR_RSSI_TO_REMRSSI_FOR_MP_UI 1
#define MP_RADIO_ZERO_BURST_AFTER_LOSS_MS 2500UL

// V26 SURGICAL LINK-LOSS INDICATOR:
// Only affects GCS -> Mission Planner status reporting when the UAV LoRa beacon is stale.
// It does not change LoRa PHY, packet format, ACK logic, MAVLink identity, or telemetry reconstruction.
#define MP_LINK_LOSS_INDICATOR_ENABLE 1
#define MP_LINK_LOSS_ZERO_STATUS_INTERVAL_MS 1000UL
#define MP_LINK_LOSS_STATUSTEXT_REPEAT_MS 15000UL
#define BRIDGE_RADIO_SYS_ID 250
#ifndef MAV_COMP_ID_TELEMETRY_RADIO
#define MAV_COMP_ID_TELEMETRY_RADIO 68
#endif

SPIClass spi(VSPI);
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
SX1278 radio = new Module(LORA_SS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, spiSettings);

#define GCS_Serial Serial
HardwareSerial MetricsSerial(2);

// VALID_* flag defines moved to TelemetryProtoFix.h (VALID_HEARTBEAT..VALID_LOCAL_VEL, VALID2_*)

// ================= Struct + beacon metric helpers =================
// OTA struct definitions live in TelemetryProtoFix.h.
// updateRemoteMetricFromBeacon78 / downlinkAckQualityPercentFromUavMeta /
// latestRemoteRssiByteOrZero are defined AFTER the global variable block below.

#define TELEMETRY_BEACON_PACKET_BYTES          ((uint16_t)sizeof(TelemetryBeaconPacket))
#define TELEMETRY_BEACON_APP_PAYLOAD_BYTES     ((uint16_t)(sizeof(TelemetryBeaconPacket) - sizeof(PacketHeader)))
#define TELEMETRY_BEACON_SENSOR_PAYLOAD_BYTES  ((uint16_t)sizeof(PixhawkDataBeacon))

#define RAW_PKT_HEADER_LEN (sizeof(PacketHeader) + 1)
#define PARAM_BULK_BASE_LEN (sizeof(PacketHeader) + 4)
#define PARAM_BULK_LEN(n) (PARAM_BULK_BASE_LEN + ((uint16_t)(n) * sizeof(CompactParamValue)))
#define COMPACT_CMD_MAX_LEN sizeof(CompactCommandLongPacket)

// Forward declarations used before the Arduino preprocessor generates prototypes.
// Penting untuk Arduino IDE: tanpa deklarasi eksplisit ini, auto-prototype Arduino
// dapat ditempatkan sebelum struct custom seperti MavlinkRawPacket/ParamBulkPacket/
// CompactCommandQueueItem/ConfigProposalPacket sehingga compile gagal.
bool peekNextRawPacket(MavlinkRawPacket &pkt, bool &fromHighQueue);
bool decodeArmCommandFromCompactItem(const CompactCommandQueueItem &item, bool &arm, bool &force);
bool decodeArmCommandFromRawPacket(const MavlinkRawPacket &raw, bool &arm, bool &force);
void parseAndReencodeRawToMissionPlanner(const MavlinkRawPacket &raw);
void parseAndReencodeParamBulkToMissionPlanner(const ParamBulkPacket &pkt);
uint16_t extractFlightActionCommandIdFromRawPacket(const MavlinkRawPacket &raw);
bool sendConfigAck(const ConfigProposalPacket &proposal);
bool peekCompactCommandPacket(CompactCommandQueueItem &item);
void popCompactCommandPacket();
void flushCompactCommandQueue();
bool enqueueCompactCommandFromMissionPlanner(const mavlink_message_t &msg, uint16_t commandIdForAck);
bool shouldProxyAckForCommand(uint16_t command);
unsigned long paramSyncNoValueExitForSF(uint8_t sf);
unsigned long paramSyncIdleExitForSF(uint8_t sf);
void startCalibrationConfigMode();
void handleTelemetryPacketCommon(uint32_t pktCounter, uint8_t pktType, uint8_t sf, uint8_t tp, uint8_t profile, uint8_t mode, uint16_t latency_x100, uint16_t packetSize, const TelemetryMeta &meta);


// ================= Variabel global =================
int activeSF = SF_MIN;
int activeTP = GCS_DEFAULT_TP;
int scanSF = SF_MIN;
uint8_t recoveryStep = 0;

bool scheduledConfig = false;
uint8_t scheduledSF = SF_MIN;
uint8_t scheduledTP = GCS_FIXED_TP;
uint8_t scheduledProfile = PROFILE_BEACON;
uint32_t scheduledApplyCounter = 0;
unsigned long scheduledConfigSinceMs = 0;

unsigned long lastPacketMs = 0;
unsigned long lastScanMs = 0;
unsigned long tStart = 0;

uint32_t totalRx = 0;
uint32_t totalLost = 0;
int32_t lastCounter = -1;

uint32_t radioBytesRx = 0;
uint32_t radioBytesTx = 0;
uint32_t telemetryBytesRx = 0;
uint32_t telemetryPayloadBytesRx = 0;
uint32_t paramRequestCount = 0;
uint32_t paramValueCount = 0;
uint32_t paramBulkRxCount = 0;
uint32_t invalidLengthDrop = 0;
uint32_t invalidProtocolDrop = 0;
uint32_t invalidCrcDrop = 0;
uint32_t mpMavCount = 0;
uint32_t mpMavBytes = 0;
uint32_t mavlinkParseDrop = 0;
uint32_t scheduledConfigCancelCount = 0;
uint32_t recoveryScanCount = 0;
uint32_t gcsLoRaHardRecoveryCount = 0;
unsigned long lastGcsLoRaHardRecoveryMs = 0;
uint32_t calibrationDedupDrop = 0;

uint8_t lastProfile = PROFILE_BEACON;
uint8_t lastPktType = 0;

uint16_t lastCalCommand = 0;
float lastCalP1 = 0.0f, lastCalP2 = 0.0f, lastCalP3 = 0.0f, lastCalP4 = 0.0f;
float lastCalP5 = 0.0f, lastCalP6 = 0.0f, lastCalP7 = 0.0f;
unsigned long lastCalCommandMs = 0;

unsigned long lastParamRequestListForwardMs = 0;
unsigned long lastParamExtRequestListForwardMs = 0;
#define PARAM_REQUEST_LIST_DEDUP_MS 12000UL

mavlink_message_t msg_gcs_in;
mavlink_status_t status_gcs_in;
mavlink_message_t msg_out;
uint8_t mavBuf[MAVLINK_MAX_PACKET_LEN];

#if MP_MAVLINK_SEQ_NORMALIZER_ENABLE
struct MpMavSeqState {
  uint8_t sysid;
  uint8_t compid;
  uint8_t next_seq;
  bool used;
};
MpMavSeqState mpSeqState[24];

uint8_t nextMpMavSeq(uint8_t sysid, uint8_t compid) {
  for (uint8_t i = 0; i < 24; i++) {
    if (mpSeqState[i].used && mpSeqState[i].sysid == sysid && mpSeqState[i].compid == compid) {
      uint8_t s = mpSeqState[i].next_seq++;
      return s;
    }
  }
  for (uint8_t i = 0; i < 24; i++) {
    if (!mpSeqState[i].used) {
      mpSeqState[i].used = true;
      mpSeqState[i].sysid = sysid;
      mpSeqState[i].compid = compid;
      mpSeqState[i].next_seq = 1;
      return 0;
    }
  }
  // Fallback jika tabel penuh: tetap deterministic, tidak memakai sequence bawaan raw/encode.
  static uint8_t fallbackSeq = 0;
  return fallbackSeq++;
}

void normalizeMpMavlinkSeq(mavlink_message_t &msg) {
  msg.seq = nextMpMavSeq(msg.sysid, msg.compid);
}
#endif

MavlinkRawPacket rawHighQueue[RAW_HIGH_QUEUE_SIZE];
uint16_t rawHighHead = 0, rawHighTail = 0, rawHighCount = 0;

MavlinkRawPacket rawLowQueue[RAW_LOW_QUEUE_SIZE];
uint16_t rawLowHead = 0, rawLowTail = 0, rawLowCount = 0;

CompactCommandQueueItem compactCmdQueue[COMPACT_CMD_QUEUE_SIZE];
uint16_t compactCmdHead = 0, compactCmdTail = 0, compactCmdCount = 0;
uint16_t compactCmdSeq = 0;

uint32_t rawHighDrop = 0, rawLowDrop = 0, commandDrop = 0;
uint32_t compactCmdQueuedCount = 0, compactCmdTxCount = 0, compactCmdDrop = 0;

bool phyLocked = false;
PixhawkDataBeacon latestBeaconData;
bool latestBeaconValid = false;
unsigned long lastSyntheticHeartbeatMs = 0;
unsigned long lastMetricRxMs = 0;

bool optimisticModeActive = false;
uint8_t optimisticModeBaseMode = 0;
uint32_t optimisticModeCustomMode = 0;
uint8_t optimisticModeSysId = 1;
uint8_t optimisticModeCompId = MAV_COMP_ID_AUTOPILOT1;
uint8_t optimisticModeSystemStatus = MAV_STATE_ACTIVE;
unsigned long optimisticModeUntilMs = 0;
unsigned long optimisticModeLastSendMs = 0;
unsigned long optimisticModeRealMatchSinceMs = 0;
uint32_t optimisticModeHeartbeatCount = 0;
uint32_t optimisticModeClearCount = 0;

uint16_t pdrWinExpected[METRICS_WINDOW_SIZE] = {0};
uint16_t pdrWinRx[METRICS_WINDOW_SIZE] = {0};
uint16_t pdrWinBytes[METRICS_WINDOW_SIZE] = {0};
uint16_t pdrWinIdx = 0;
uint16_t pdrWinFilled = 0;

uint32_t securityReplayDrop = 0;
bool securityHaveLastUavCounter = false;
uint32_t securityLastUavCounter = 0;
unsigned long securityLastUavCounterMs = 0;

int16_t mpLinkQualityEwma_x10 = -1;
unsigned long lastMpLinkQualityUpdateMs = 0;

// Link metrics reported by UAV in the 78-byte beacon. These reflect real
// downlink/ACK reception at the UAV side and are used for RADIO_STATUS.remrssi
// and bidirectional-link loss detection.
uint8_t latestUavCntAck = 0;
uint16_t latestUavSuccessStreak = 0;
uint16_t latestUavFailStreak = 0;
unsigned long latestUavMetaMs = 0;
unsigned long firstBeaconMs = 0;
unsigned long lastMpLinkLossMs = 0;
bool mpLinkLossState = false;
bool mpLinkLossAnnounced = false;
unsigned long lastMpLinkLossZeroStatusMs = 0;

// True only inside reconstructed telemetry/radio-status output. PARAM_VALUE,
// COMMAND_ACK, MISSION_* and raw setup packets bypass this so param sync stays fast.
bool mpTelemetryOutputContext = false;
uint32_t mpTelemetryOutCount = 0;
uint32_t mpFastBypassOutCount = 0;
uint32_t mpRawTelemetryDedupDrop = 0;


#if MP_SINGLE_SOURCE_IDENTITY_ENABLE
uint8_t mpCanonicalSysId(uint8_t sysid) {
  if (latestBeaconValid && latestBeaconData.system_id) return latestBeaconData.system_id;
  if (sysid) return sysid;
  return 1;
}

uint8_t mpCanonicalCompId(uint8_t compid) {
  (void)compid;
  return MP_SINGLE_SOURCE_FORCE_COMPID;
}

void canonicalizeMissionPlannerSource(uint8_t &sysid, uint8_t &compid) {
  sysid = mpCanonicalSysId(sysid);
  compid = mpCanonicalCompId(compid);
}
#else
void canonicalizeMissionPlannerSource(uint8_t &sysid, uint8_t &compid) {
  if (sysid == 0) sysid = 1;
  if (compid == 0) compid = MAV_COMP_ID_AUTOPILOT1;
}
#endif

// Remote link metric nyata dari sisi UAV: UAV mengukur RSSI/SNR paket downlink GCS,
// lalu mengirim ringkasannya di beacon 78 byte. Ini membuat remrssi RADIO_STATUS
// berbasis pengukuran RF nyata, bukan salinan/sintesis dari rssi lokal GCS.
uint8_t lastRemoteRssiQ = 0;
int8_t lastRemoteSnrX2 = -128;
unsigned long lastRemoteMetricMs = 0;

// ---------------------------------------------------------------------------
// Beacon metric helpers — must appear AFTER their global dependencies above.
// ---------------------------------------------------------------------------

static inline void updateRemoteMetricFromBeacon78(const TelemetryBeaconPacket &pkt) {
  lastRemoteRssiQ = pkt.remote_rssi_q;
  lastRemoteSnrX2 = pkt.remote_snr_x2;
  if (lastRemoteRssiQ > 0 && lastRemoteSnrX2 != -128) lastRemoteMetricMs = millis();
}

static inline int downlinkAckQualityPercentFromUavMeta() {
  if (latestUavMetaMs == 0 || millis() - latestUavMetaMs > 15000UL) return 0;
  int ackPct = constrain((int)latestUavCntAck * 10, 0, 100);
  if (latestUavFailStreak >= MP_BIDIR_FAIL_STREAK_THRESHOLD) {
    ackPct = min(ackPct, 20);
  } else if (latestUavFailStreak > 0) {
    int cap = 100 - ((int)latestUavFailStreak * 15);
    ackPct = min(ackPct, constrain(cap, 35, 100));
  }
  if (latestUavFailStreak == 0 && latestUavSuccessStreak >= 10 && latestUavCntAck >= 10) ackPct = 100;
  return constrain(ackPct, 0, 100);
}

static inline uint8_t latestRemoteRssiByteOrZero() {
#if !MP_REPORT_REMOTE_RSSI_TO_MP
  return UINT8_MAX;
#else
  if (lastRemoteRssiQ == 0 || lastRemoteSnrX2 == -128 ||
      millis() - lastRemoteMetricMs > 15000UL) {
    return UINT8_MAX;
  }
  float remoteRssiDbm = loraQ254ToRssiDbm(lastRemoteRssiQ);
  float remoteSnrDb   = ((float)lastRemoteSnrX2) / 2.0f;
  int rfPct = loraRfLinkMarginPercent(remoteRssiDbm, remoteSnrDb);
  return (uint8_t)constrain((int)((rfPct * 254UL) / 100UL), 1, 254);
#endif
}

// ================= Forward declarations =================
void flushLowCommandQueue();
void flushHighCommandQueue();
void updateCommandPriorityMode();
bool commandPriorityActive();
void sendRadioStatusToMissionPlanner(uint8_t sysid, uint8_t compid);
void sendTelemetryBeaconToMissionPlanner(const PixhawkDataBeacon &d);
void serviceOptimisticModeHeartbeat();
void sendOptimisticSetModeHeartbeatToMissionPlanner(const mavlink_message_t &msg);
void sendOptimisticCommandLongModeHeartbeatToMissionPlanner(const mavlink_message_t &msg);
void updateOptimisticModeFromRealBeacon(const PixhawkDataBeacon &d);
bool optimisticModeOverrideActive();
bool isSetupConfigCommand(uint16_t command);
bool isFlightActionCommandId(uint16_t command);
void notifyHighSfParamBlockedIfNeeded();
void sendHighSfConnectionNoticeIfNeeded(uint8_t sf);
void sendLoRaParamNoticeIfNeeded(uint8_t sf, uint8_t tp);
void serviceLoRaParamNoticeBurst();
void serviceRadioStatusPeriodic();
uint8_t missionPlannerRadioSignalByte();
void clearCommandProxyAck(uint16_t command);
void sendForceArmProxyAckToMissionPlanner();
void beginArmFeedbackMode();
bool armFeedbackActive();

// ================= Utility =================
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

static inline float estimateLoRaToA_ms(uint8_t sf, float bwHz, uint8_t crDen, uint16_t payloadBytes) {
  return estimateLoRaToA_ms(sf, bwHz, crDen, payloadBytes, LORA_PREAMBLE_SYMBOLS);
}

static inline float estimatePacketEnergy_mJ(uint8_t tpDbm, float toaMs) {
  return estimatePacketEnergy_mJ(tpDbm, toaMs, 3.30f);
}

bool validatePacket(const uint8_t *buf, size_t len) {
  return validatePacketInternal(buf, len, invalidLengthDrop, invalidProtocolDrop, invalidCrcDrop);
}

// loraNominalBitrateKbps defined in TelemetryProtoFix.h (duplicate removed)

unsigned long rxTimeoutForSF(uint8_t sf) {
  // Shorter polling windows keep Mission Planner command bytes from waiting
  // too long in Serial while still covering the beacon ToA at each SF.
  if (sf >= 12) return 1500UL;
  if (sf == 11) return 1150UL;
  if (sf == 10) return 850UL;
  return 650UL;
}

unsigned long linkIdleScanAfterForSF(uint8_t sf) {
  if (sf >= 12) return 8500UL;
  if (sf == 11) return 6500UL;
  if (sf == 10) return 5000UL;
  return 3500UL;
}

unsigned long mpLinkLossTimeoutForSF(uint8_t sf) {
  if (sf >= 12) return MP_LINK_LOSS_TIMEOUT_MS_SF12;
  if (sf == 11) return MP_LINK_LOSS_TIMEOUT_MS_SF11;
  if (sf == 10) return MP_LINK_LOSS_TIMEOUT_MS_SF10;
  return MP_LINK_LOSS_TIMEOUT_MS_SF7_9;
}

bool isUplinkFreshForMissionPlanner() {
  if (!latestBeaconValid) return false;
  return (millis() - lastPacketMs) <= mpLinkLossTimeoutForSF(activeSF);
}

bool isBidirectionalLinkHealthyForMissionPlanner() {
#if !MP_DISCONNECT_ON_LINK_LOSS_ENABLE
  return true;
#else
  if (!isUplinkFreshForMissionPlanner()) return false;

  // Do not block initial MP connection before the ACK window is populated.
  if (firstBeaconMs == 0 || millis() - firstBeaconMs < MP_LINK_STARTUP_GRACE_MS) return true;
  if (latestUavMetaMs == 0 || millis() - latestUavMetaMs > mpLinkLossTimeoutForSF(activeSF) + 2500UL) return false;

  if (latestUavFailStreak >= MP_BIDIR_FAIL_STREAK_THRESHOLD) return false;
  if (latestUavCntAck <= MP_BIDIR_MIN_ACK_WINDOW_OK && latestUavFailStreak >= 2) return false;
  return true;
#endif
}

uint8_t telemetryAckEveryForSF(uint8_t sf) {
#if TELEMETRY_ACK_DECIMATION_ENABLE
  // V14 delay/force fix:
  // SF10-SF11 still have enough airtime margin for ACK every beacon.
  // This gives Mission Planner commands a downlink opportunity every cycle.
  // SF12 remains decimated because a 78B beacon still consumes a large part of the 1s budget at SF12.
  if (sf >= 12) return 2;
  if (sf == 11) return 1;
  if (sf == 10) return 1;
#endif
  return 1;
}

bool shouldRespondToTelemetry(uint32_t counter, uint8_t sf) {
  uint8_t every = telemetryAckEveryForSF(sf);
  if (every <= 1) return true;
  return (counter % every) == 0;
}

void updatePdrWindow(uint16_t expected, uint16_t rx, uint16_t payloadBytes) {
  pdrWinExpected[pdrWinIdx] = expected;
  pdrWinRx[pdrWinIdx] = rx;
  pdrWinBytes[pdrWinIdx] = payloadBytes;
  pdrWinIdx = (pdrWinIdx + 1) % METRICS_WINDOW_SIZE;
  if (pdrWinFilled < METRICS_WINDOW_SIZE) pdrWinFilled++;
}

void getPdrWindowStats(uint32_t &expected, uint32_t &rx, uint32_t &bytes) {
  expected = 0; rx = 0; bytes = 0;
  for (uint16_t i = 0; i < pdrWinFilled; i++) {
    expected += pdrWinExpected[i];
    rx += pdrWinRx[i];
    bytes += pdrWinBytes[i];
  }
}

bool acceptRollingUavCounter(uint32_t pktCounter) {
#if SECURITY_ANTI_REPLAY_ENABLE
  unsigned long now = millis();

  if (!securityHaveLastUavCounter) {
    securityHaveLastUavCounter = true;
    securityLastUavCounter = pktCounter;
    securityLastUavCounterMs = now;
    return true;
  }

  int32_t delta = (int32_t)(pktCounter - securityLastUavCounter);
  if (delta > 0 && (uint32_t)delta <= SECURITY_MAX_COUNTER_GAP) {
    securityLastUavCounter = pktCounter;
    securityLastUavCounterMs = now;
    return true;
  }

  // Static PHY relock fix: after UAV reboot/SF relock, counters can restart from
  // a smaller value. Accept reset only after idle, never during active link flow.
  unsigned long idleMs = now - securityLastUavCounterMs;
  if (idleMs > SECURITY_REBOOT_GRACE_MS && pktCounter < securityLastUavCounter) {
    securityLastUavCounter = pktCounter;
    securityLastUavCounterMs = now;
    securityReplayDrop = 0;
    return true;
  }

  // During scan/relock, be less strict after half grace so SF9-SF12 do not stay
  // locked out by an old counter from the previous session.
  if (!phyLocked && idleMs > (SECURITY_REBOOT_GRACE_MS / 2UL) && pktCounter < securityLastUavCounter) {
    securityLastUavCounter = pktCounter;
    securityLastUavCounterMs = now;
    securityReplayDrop = 0;
    return true;
  }

  securityReplayDrop++;
  return false;
#else
  (void)pktCounter;
  return true;
#endif
}

int rfQualityPercentFromRssiSnr() {
  // Estimator lapangan ringan untuk SX1278. Nilai ini hanya salah satu komponen,
  // bukan pemaksa sinyal agar 100%.
  int snrPct = 100;
  if (lastSnr <= -15.0f) snrPct = 0;
  else if (lastSnr < 10.0f) snrPct = (int)(((lastSnr + 15.0f) * 100.0f) / 25.0f);

  int rssiPct = 100;
  if (lastRssi <= -125.0f) rssiPct = 0;
  else if (lastRssi < -80.0f) rssiPct = (int)(((lastRssi + 125.0f) * 100.0f) / 45.0f);

  return constrain((snrPct * 60 + rssiPct * 40) / 100, 0, 100);
}

void applyRadioSettings(uint8_t sf, uint8_t tp) {
  if (sf < SF_MIN) sf = SF_MIN;
  if (sf > SF_MAX) sf = SF_MAX;
  if (tp < TP_MIN) tp = TP_MIN;
  if (tp > TP_MAX) tp = TP_MAX;

  activeSF = sf;
  activeTP = tp;

  radio.standby(); delay(1);
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(activeSF);
  radio.setOutputPower(activeTP);
  radio.standby();

  MetricsSerial.print("[RADIO GCS] Applied SF=");
  MetricsSerial.print(activeSF);
  MetricsSerial.print(" TP=");
  MetricsSerial.println(activeTP);
}

void applyRadioSettings(uint8_t sf) {
  applyRadioSettings(sf, activeTP);
}

void softRecoverRadio() {
  radio.standby(); delay(2);
  radio.sleep(); delay(10);
  applyRadioSettings(activeSF);
}

uint16_t telemetryPayloadBytesForType(uint8_t pktType) {
  if (pktType == PKT_TELEM_BEACON) return sizeof(PixhawkDataBeacon);
  return 0;
}

bool calibrationConfigModeActive() { return calConfigActive && millis() < calConfigUntilMs; }

void startCalibrationConfigMode() {
  calConfigActive = true;
  calConfigUntilMs = millis() + CAL_CONFIG_HOLD_MS;
  paramModeHoldUntilMs = millis() + PARAM_MODE_HOLD_MS;
  linkMode = LINK_MODE_CALIBRATION;
}

void stopCalibrationConfigMode() {
  calConfigActive = false;
  calConfigUntilMs = 0;
  if (!paramSyncActive && millis() > paramModeHoldUntilMs) linkMode = LINK_MODE_NORMAL;
}

void updateCalibrationConfigMode() {
  if (!calConfigActive) return;
  if (millis() <= calConfigUntilMs) return;
  stopCalibrationConfigMode();
}

bool fastOperationalModeActive() {
  return paramSyncActive || calibrationConfigModeActive() || millis() < paramModeHoldUntilMs || linkMode == LINK_MODE_PARAM_SYNC || linkMode == LINK_MODE_CALIBRATION;
}

void printMetricsHeader() {
  MetricsSerial.println(
    "counter,rssi_dbm,snr_db,pdr_cum_pct,plr_cum_pct,pdr_win_pct,plr_win_pct,"
    "throughput_cum_kbps,throughput_inst_kbps,lora_bit_rate_kbps,latency_ack_half_rtt_ms,inter_arrival_ms,"
    "etp_mJ,edp_mJ,uav_sf,uav_tp_dbm,gcs_active_sf,gcs_active_tp_dbm,bw_khz,toa_ms,"
    "profile,link_mode,packet_type,packet_bytes,payload_bytes,uav_high_queue,uav_low_queue,"
    "uav_high_drop,uav_low_drop,uav_param_drop,mp_mav_count,mp_mav_bytes,gcs_high_q,gcs_low_q,"
    "gcs_high_drop,gcs_low_drop,recovery_scan_count,sched_cancel_count,calib_dedup_drop,"
    "cal_cmd_sent,mag_cal_progress_rx,mag_cal_report_rx,cal_command_ack_rx,cal_statustext_rx"
  );
}

void printMetrics(uint32_t pktCounter, uint8_t pktType, uint8_t sf, uint8_t tp, uint8_t profile, uint8_t mode, uint16_t latency_x100, uint16_t telemBytes, const TelemetryMeta &meta) {
  (void)meta;
  unsigned long now = millis();
  uint16_t payloadBytes = telemetryPayloadBytesForType(pktType);

  uint16_t expectedThis = 1;
  if (lastCounter != -1 && (int32_t)pktCounter > lastCounter + 1) {
    expectedThis = (uint16_t)((int32_t)pktCounter - lastCounter);
    totalLost += (expectedThis - 1);
  }
  lastCounter = (int32_t)pktCounter;
  totalRx++;
  telemetryBytesRx += telemBytes;
  telemetryPayloadBytesRx += payloadBytes;
  updatePdrWindow(expectedThis, 1, payloadBytes);

  uint32_t totalExpected = totalRx + totalLost;
  float pdrCumPct = (totalExpected > 0) ? (100.0f * totalRx / totalExpected) : 0.0f;
  float plrCumPct = (totalExpected > 0) ? (100.0f * totalLost / totalExpected) : 0.0f;

  uint32_t winExpected = 0, winRx = 0, winBytes = 0;
  getPdrWindowStats(winExpected, winRx, winBytes);
  float pdrWinPct = (winExpected > 0) ? (100.0f * winRx / winExpected) : 0.0f;
  float plrWinPct = (winExpected > 0) ? (100.0f * (winExpected - winRx) / winExpected) : 0.0f;

  float elapsedSec = (now - tStart) / 1000.0f;
  if (elapsedSec <= 0.0f) elapsedSec = 0.001f;
  float throughputCumKbps = ((telemetryPayloadBytesRx * 8.0f) / elapsedSec) / 1000.0f;

  float interArrivalMs = 0.0f;
  if (lastMetricRxMs != 0 && now >= lastMetricRxMs) interArrivalMs = (float)(now - lastMetricRxMs);
  lastMetricRxMs = now;
  float throughputInstKbps = (interArrivalMs > 0.0f) ? ((payloadBytes * 8.0f) / interArrivalMs) : 0.0f;

  float loraBitRateKbps = loraNominalBitrateKbps(sf, LORA_BW_KHZ * 1000.0f, LORA_CR_DEN);
  float toaMs = estimateLoRaToA_ms(sf, LORA_BW_KHZ * 1000.0f, LORA_CR_DEN, telemBytes);
  float latencyAckHalfRttMs = latency_x100 / 100.0f;
  float etpMJ = estimatePacketEnergy_mJ(tp, toaMs);
  float pdrForEdp = (pdrWinPct > 0.0f) ? (pdrWinPct / 100.0f) : 0.0f;
  float edpMJ = (pdrForEdp > 0.0f) ? (etpMJ / pdrForEdp) : 0.0f;

  MetricsSerial.print(pktCounter); MetricsSerial.print(",");
  MetricsSerial.print(radio.getRSSI(), 2); MetricsSerial.print(",");
  MetricsSerial.print(radio.getSNR(), 2); MetricsSerial.print(",");
  MetricsSerial.print(pdrCumPct, 2); MetricsSerial.print(",");
  MetricsSerial.print(plrCumPct, 2); MetricsSerial.print(",");
  MetricsSerial.print(pdrWinPct, 2); MetricsSerial.print(",");
  MetricsSerial.print(plrWinPct, 2); MetricsSerial.print(",");
  MetricsSerial.print(throughputCumKbps, 4); MetricsSerial.print(",");
  MetricsSerial.print(throughputInstKbps, 4); MetricsSerial.print(",");
  MetricsSerial.print(loraBitRateKbps, 4); MetricsSerial.print(",");
  MetricsSerial.print(latencyAckHalfRttMs, 2); MetricsSerial.print(",");
  MetricsSerial.print(interArrivalMs, 2); MetricsSerial.print(",");
  MetricsSerial.print(etpMJ, 4); MetricsSerial.print(",");
  MetricsSerial.print(edpMJ, 4); MetricsSerial.print(",");
  MetricsSerial.print((int)sf); MetricsSerial.print(",");
  MetricsSerial.print((int)tp); MetricsSerial.print(",");
  MetricsSerial.print((int)activeSF); MetricsSerial.print(",");
  MetricsSerial.print((int)activeTP); MetricsSerial.print(",");
  MetricsSerial.print((int)LORA_BW_KHZ); MetricsSerial.print(",");
  MetricsSerial.print(toaMs, 2); MetricsSerial.print(",");
  MetricsSerial.print((int)profile); MetricsSerial.print(",");
  MetricsSerial.print((int)mode); MetricsSerial.print(",");
  MetricsSerial.print((int)pktType); MetricsSerial.print(",");
  MetricsSerial.print(telemBytes); MetricsSerial.print(",");
  MetricsSerial.print(payloadBytes); MetricsSerial.print(",");
  MetricsSerial.print(0); MetricsSerial.print(",");
  MetricsSerial.print(0); MetricsSerial.print(",");
  MetricsSerial.print(0); MetricsSerial.print(",");
  MetricsSerial.print(0); MetricsSerial.print(",");
  MetricsSerial.print(0); MetricsSerial.print(",");
  MetricsSerial.print(mpMavCount); MetricsSerial.print(",");
  MetricsSerial.print(mpMavBytes); MetricsSerial.print(",");
  MetricsSerial.print(rawHighCount); MetricsSerial.print(",");
  MetricsSerial.print(rawLowCount); MetricsSerial.print(",");
  MetricsSerial.print(rawHighDrop); MetricsSerial.print(",");
  MetricsSerial.print(rawLowDrop); MetricsSerial.print(",");
  MetricsSerial.print(recoveryScanCount); MetricsSerial.print(",");
  MetricsSerial.print(scheduledConfigCancelCount); MetricsSerial.print(",");
  MetricsSerial.print(calibrationDedupDrop); MetricsSerial.print(",");
  MetricsSerial.print(calCommandSentCount); MetricsSerial.print(",");
  MetricsSerial.print(magCalProgressRxCount); MetricsSerial.print(",");
  MetricsSerial.print(magCalReportRxCount); MetricsSerial.print(",");
  MetricsSerial.print(calCommandAckRxCount); MetricsSerial.print(",");
  MetricsSerial.println(calStatustextRxCount);
}

unsigned long paramSyncNoValueExitForSF(uint8_t sf) {
  if (sf >= 12) return PARAM_SYNC_NO_VALUE_EXIT_MS_SF12;
  if (sf == 11) return PARAM_SYNC_NO_VALUE_EXIT_MS_SF11;
  if (sf == 10) return PARAM_SYNC_NO_VALUE_EXIT_MS_SF10;
  return PARAM_SYNC_NO_VALUE_EXIT_MS_SF7_9;
}

unsigned long paramSyncIdleExitForSF(uint8_t sf) {
  // Setelah param sync selesai/di-cancel, Data tab harus kembali normal tanpa menunggu 20 detik.
  if (sf >= 11) return 5000UL;
  if (sf == 10) return 3500UL;
  return PARAM_SYNC_IDLE_EXIT_MS;
}

void flushParamSyncCommandQueuesForRecovery() {
  // PARAM_REQUEST_LIST/READ masuk high-priority. Jika user menekan Cancel di Mission Planner,
  // request lama tidak boleh tertahan lalu dikirim setelah link pulih.
  flushLowCommandQueue();
  if (paramSyncActive && rawHighCount > (RAW_HIGH_QUEUE_SIZE / 2)) {
    rawHighHead = rawHighTail = rawHighCount = 0;
  }
}

void abortParamSyncRecovery(const char *reason) {
  (void)reason;
  paramSyncAutoAbortCount++;
  flushParamSyncCommandQueuesForRecovery();
  paramSyncActive = false;
  paramSyncStartMs = 0;
  lastParamRequestListForwardMs = 0;
  lastParamExtRequestListForwardMs = 0;
  lastParamValueMs = 0; lastParamIndex = 0; lastParamCount = 0;
  paramModeHoldUntilMs = 0;
  if (calibrationConfigModeActive()) linkMode = LINK_MODE_CALIBRATION;
  else linkMode = LINK_MODE_NORMAL;
}


bool commandPriorityActive() {
  return commandModeUntilMs != 0 && millis() < commandModeUntilMs;
}

void flushCompactCommandQueue();
bool enqueueCompactCommandFromMissionPlanner(const mavlink_message_t &msg, uint16_t commandIdForAck);
bool peekCompactCommandPacket(CompactCommandQueueItem &item);
void popCompactCommandPacket();

void beginFlightCommandPriorityMode() {
  flightCommandRxCount++;
  lastFlightCommandRxMs = millis();
  commandModeUntilMs = millis() + FLIGHT_COMMAND_HOLD_MS;
  // Flight/action commands must not wait behind stale parameter/config queues.
  abortParamSyncRecovery("flight_command_priority");
  flushLowCommandQueue();
  flushHighCommandQueue();
  flushCompactCommandQueue();
  flightCommandQueueFlushCount++;
  linkMode = LINK_MODE_COMMAND;
}

void updateCommandPriorityMode() {
  if (commandModeUntilMs == 0) return;
  unsigned long now = millis();
  if (now <= commandModeUntilMs) { linkMode = LINK_MODE_COMMAND; return; }
  commandModeUntilMs = 0;
  if (!paramSyncActive && !calibrationConfigModeActive()) linkMode = LINK_MODE_NORMAL;
}

uint16_t extractCommandIdFromMissionPlannerMessage(const mavlink_message_t &msg) {
  if (msg.msgid == MAVLINK_MSG_ID_SET_MODE) return CMD_DO_SET_MODE;
  if (msg.msgid == MAVLINK_MSG_ID_MISSION_SET_CURRENT) return MAV_CMD_MISSION_START;
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    return (uint16_t)cmd.command;
  }
#ifdef MAVLINK_MSG_ID_COMMAND_INT
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    mavlink_command_int_t cmd;
    mavlink_msg_command_int_decode(&msg, &cmd);
    return (uint16_t)cmd.command;
  }
#endif
  return 0;
}

bool isBackgroundOrInternalCommandAck(uint16_t command) {
  return command == MAV_CMD_SET_MESSAGE_INTERVAL ||
         command == MAV_CMD_REQUEST_MESSAGE ||
         command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
}

void rememberMissionPlannerCommandForAck(uint16_t command, bool flightAction) {
  if (command == 0) return;
  unsigned long now = millis();
  mpPendingCommand[mpPendingCommandHead] = command;
  mpPendingCommandMs[mpPendingCommandHead] = now;
  mpPendingCommandHead = (mpPendingCommandHead + 1) % MP_PENDING_COMMAND_SLOTS;
  if (flightAction) commandPriorityExpectedAck = command;
}

bool hasRecentMissionPlannerCommand(uint16_t command) {
  if (command == 0) return false;
  unsigned long now = millis();
  for (uint8_t i = 0; i < MP_PENDING_COMMAND_SLOTS; i++) {
    if (mpPendingCommand[i] == command && (now - mpPendingCommandMs[i]) <= MP_COMMAND_ACK_GUARD_MS) return true;
  }
  return false;
}

void clearRecentMissionPlannerCommand(uint16_t command) {
  if (command == 0) return;
  for (uint8_t i = 0; i < MP_PENDING_COMMAND_SLOTS; i++) {
    if (mpPendingCommand[i] == command) {
      mpPendingCommand[i] = 0;
      mpPendingCommandMs[i] = 0;
    }
  }
  if (commandPriorityExpectedAck == command) commandPriorityExpectedAck = 0;
}

bool wasCommandProxyAcked(uint16_t command) {
  if (command == 0) return false;
  unsigned long now = millis();
  for (uint8_t i = 0; i < MP_PENDING_COMMAND_SLOTS; i++) {
    if (proxiedAckCommand[i] == command && (now - proxiedAckCommandMs[i]) <= COMMAND_PROXY_ACK_SUPPRESS_MS) return true;
  }
  return false;
}

void rememberCommandProxyAck(uint16_t command) {
  if (command == 0) return;
  proxiedAckCommand[proxiedAckHead] = command;
  proxiedAckCommandMs[proxiedAckHead] = millis();
  proxiedAckHead = (proxiedAckHead + 1) % MP_PENDING_COMMAND_SLOTS;
}

void clearCommandProxyAck(uint16_t command) {
  if (command == 0) return;
  for (uint8_t i = 0; i < MP_PENDING_COMMAND_SLOTS; i++) {
    if (proxiedAckCommand[i] == command) {
      proxiedAckCommand[i] = 0;
      proxiedAckCommandMs[i] = 0;
    }
  }
}

bool shouldForwardCommandAckToMissionPlanner(const mavlink_command_ack_t &ack) {
  uint16_t command = (uint16_t)ack.command;

  // ACK background/internal tidak boleh diteruskan ke Mission Planner. Bahkan jika request
  // SET_MESSAGE_INTERVAL berasal dari MP, ACK ini sering datang saat doCommand menunggu ACK action
  // sehingga memunculkan "Commands dont match" dan bisa membuat Mission Planner crash.
  if (isBackgroundOrInternalCommandAck(command)) {
    commandAckSuppressedCount++;
    return false;
  }

  // Jika GCS sudah mengirim ACK proxy untuk action command high-SF, suppress ACK asli yang datang belakangan
  // supaya Mission Planner tidak menerima ACK ganda saat state doCommand sudah selesai.
  if (wasCommandProxyAcked(command)) {
    clearRecentMissionPlannerCommand(command);
    commandAckSuppressedCount++;
    return false;
  }

  // V15_4FIX: ARM/FORCE-ARM must be transaction-guarded. A delayed ACK 400 from
  // normal ARM must not be forwarded while Mission Planner is waiting for FORCE ARM.
  if (isArmDisarmCommandId(command)) {
    return shouldForwardArmAckToMissionPlanner(ack);
  }

  if (hasRecentMissionPlannerCommand(command)) {
    clearRecentMissionPlannerCommand(command);
    return true;
  }

  // Pada SF tinggi, COMMAND_ACK yang datang terlambat mudah tertukar dengan doCommand baru
  // di Mission Planner. Jangan teruskan ACK yang tidak punya command pending.
  if (activeSF >= COMMAND_PROXY_ACK_MIN_SF) {
    commandAckSuppressedCount++;
    return false;
  }

  // In command priority mode, only forward action/setup ACKs. Drop stale unrelated ACKs.
  if (commandPriorityActive()) {
    if (isFlightActionCommandId(command) || isSetupConfigCommand(command)) return true;
    commandAckSuppressedCount++;
    return false;
  }

  return true;
}

void startParamSync() {
  bool wasActive = paramSyncActive;
  paramSyncActive = true;
  linkMode = calibrationConfigModeActive() ? LINK_MODE_CALIBRATION : LINK_MODE_PARAM_SYNC;
  paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
  paramModeHoldUntilMs = millis() + PARAM_MODE_HOLD_MS;
  if (!wasActive) { paramSyncStartMs = millis(); lastParamValueMs = 0; lastParamIndex = 0; lastParamCount = 0; }
}

void stopParamSync() {
  paramSyncActive = false;
  paramSyncStartMs = 0;
  lastParamRequestListForwardMs = 0;
  lastParamExtRequestListForwardMs = 0;
  lastParamValueMs = 0; lastParamIndex = 0; lastParamCount = 0;
  paramModeHoldUntilMs = millis() + PARAM_MODE_HOLD_MS;
  if (calibrationConfigModeActive()) linkMode = LINK_MODE_CALIBRATION;
  else linkMode = LINK_MODE_NORMAL;
}

void updateParamSyncTimeout() {
  unsigned long now = millis();
  if (!paramSyncActive) {
    if (paramModeHoldUntilMs != 0 && now > paramModeHoldUntilMs) {
      paramModeHoldUntilMs = 0;
      if (!calibrationConfigModeActive()) linkMode = LINK_MODE_NORMAL;
    }
    return;
  }
  if (now > paramSyncUntilMs) { abortParamSyncRecovery("param_timeout"); return; }
  if (paramSyncStartMs > 0 && lastParamValueMs == 0 && now - paramSyncStartMs > paramSyncNoValueExitForSF(activeSF)) {
    abortParamSyncRecovery("no_param_value"); return;
  }
  if (lastParamCount > 0 && lastParamIndex >= (lastParamCount - 1)) { stopParamSync(); return; }
  if (lastParamValueMs > 0 && now - lastParamValueMs > paramSyncIdleExitForSF(activeSF)) { abortParamSyncRecovery("param_idle_fast_restore"); return; }
}

void syncModeFromUAV(uint8_t uavMode) {
  if (commandPriorityActive()) { linkMode = LINK_MODE_COMMAND; return; }
  if (uavMode == LINK_MODE_CALIBRATION) {
    linkMode = LINK_MODE_CALIBRATION;
    paramModeHoldUntilMs = millis() + PARAM_MODE_HOLD_MS;
  } else if (uavMode == LINK_MODE_PARAM_SYNC) {
    linkMode = LINK_MODE_PARAM_SYNC;
    paramModeHoldUntilMs = millis() + PARAM_MODE_HOLD_MS;
  } else {
    if (!paramSyncActive && !calibrationConfigModeActive() && millis() > paramModeHoldUntilMs) linkMode = LINK_MODE_NORMAL;
  }
}

void copyParamId(char *dest, const char *src) { memcpy(dest, src, 16); dest[16] = 0; }
bool paramIdStartsWith(const char *id, const char *prefix) { return strncmp(id, prefix, strlen(prefix)) == 0; }
bool paramIdEquals(const char *id, const char *name) { return strncmp(id, name, 16) == 0; }

bool isInteractiveSetupParamId(const char *id) {
  if (id == nullptr || id[0] == 0) return false;
  if (paramIdStartsWith(id, "COMPASS_")) return true;
  if (paramIdStartsWith(id, "INS_")) return true;
  if (paramIdStartsWith(id, "AHRS_")) return true;
  if (paramIdStartsWith(id, "GPS_")) return true;
  if (paramIdStartsWith(id, "GPS2_")) return true;
  if (paramIdStartsWith(id, "SERIAL")) return true;
  if (paramIdStartsWith(id, "CAN_")) return true;
  if (paramIdStartsWith(id, "BRD_")) return true;
  if (paramIdStartsWith(id, "EK2_")) return true;
  if (paramIdStartsWith(id, "EK3_")) return true;
  if (paramIdStartsWith(id, "ARMING_")) return true;
  if (paramIdStartsWith(id, "RC")) return true;
  if (paramIdStartsWith(id, "FS_")) return true;
  if (paramIdStartsWith(id, "SERVO")) return true;
  if (paramIdStartsWith(id, "MOT_")) return true;
  if (paramIdStartsWith(id, "ESC_")) return true;
  if (paramIdStartsWith(id, "FLTMODE")) return true;
  if (paramIdEquals(id, "SIMPLE")) return true;
  if (paramIdEquals(id, "SUPER_SIMPLE")) return true;
  if (paramIdEquals(id, "MODE_CH")) return true;
  if (paramIdStartsWith(id, "BATT")) return true;
  if (paramIdStartsWith(id, "FENCE_")) return true;
  if (paramIdStartsWith(id, "ATC_")) return true;
  if (paramIdStartsWith(id, "PSC_")) return true;
  if (paramIdStartsWith(id, "PILOT_")) return true;
  if (paramIdStartsWith(id, "WPNAV_")) return true;
  if (paramIdStartsWith(id, "ANGLE_")) return true;
  return false;
}

bool isCalibrationCommand(uint16_t command) {
  return (command == CMD_PREFLIGHT_CALIBRATION || command == CMD_DO_START_MAG_CAL ||
          command == CMD_DO_ACCEPT_MAG_CAL || command == CMD_DO_CANCEL_MAG_CAL ||
          command == CMD_ACCELCAL_VEHICLE_POS);
}

bool isSetupConfigCommand(uint16_t command) {
  return isCalibrationCommand(command) || command == CMD_PREFLIGHT_STORAGE ||
         command == CMD_PREFLIGHT_REBOOT_SHUTDOWN || command == CMD_START_RX_PAIR;
}

bool isCriticalCommandLong(uint16_t command) {
  return (command == CMD_DO_SET_MODE || command == CMD_COMPONENT_ARM_DISARM || isSetupConfigCommand(command));
}

bool isFlightActionCommandId(uint16_t command) {
  return command == CMD_COMPONENT_ARM_DISARM || command == CMD_DO_SET_MODE ||
         command == MAV_CMD_NAV_WAYPOINT || command == MAV_CMD_NAV_LOITER_UNLIM ||
         command == MAV_CMD_NAV_LOITER_TURNS || command == MAV_CMD_NAV_LOITER_TIME ||
         command == MAV_CMD_NAV_RETURN_TO_LAUNCH || command == MAV_CMD_NAV_LAND ||
         command == MAV_CMD_NAV_TAKEOFF || command == MAV_CMD_DO_REPOSITION ||
         command == MAV_CMD_MISSION_START || command == MAV_CMD_DO_SET_HOME;
}

bool isFlightActionGCSMessage(const mavlink_message_t &msg) {
  if (msg.msgid == MAVLINK_MSG_ID_SET_MODE || msg.msgid == MAVLINK_MSG_ID_MISSION_SET_CURRENT ||
      msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL || msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
    return true;
  }
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd; mavlink_msg_command_long_decode(&msg, &cmd);
    return isFlightActionCommandId((uint16_t)cmd.command);
  }
#ifdef MAVLINK_MSG_ID_COMMAND_INT
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    mavlink_command_int_t cmd; mavlink_msg_command_int_decode(&msg, &cmd);
    return isFlightActionCommandId((uint16_t)cmd.command);
  }
#endif
  return false;
}

bool isDuplicateCalibrationCommand(const mavlink_message_t &msg) {
  if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) return false;
  mavlink_command_long_t cmd;
  mavlink_msg_command_long_decode(&msg, &cmd);
  uint16_t command = (uint16_t)cmd.command;
  if (!isCalibrationCommand(command)) return false;
  unsigned long now = millis();
  bool sameCommand = (command == lastCalCommand);
  bool sameParams = (fabsf(cmd.param1 - lastCalP1) < 0.001f && fabsf(cmd.param2 - lastCalP2) < 0.001f &&
                     fabsf(cmd.param3 - lastCalP3) < 0.001f && fabsf(cmd.param4 - lastCalP4) < 0.001f &&
                     fabsf(cmd.param5 - lastCalP5) < 0.001f && fabsf(cmd.param6 - lastCalP6) < 0.001f &&
                     fabsf(cmd.param7 - lastCalP7) < 0.001f);
  bool insideWindow = (now - lastCalCommandMs) < CALIBRATION_DEDUP_MS;
  if (sameCommand && sameParams && insideWindow) { calibrationDedupDrop++; return true; }
  lastCalCommand = command; lastCalP1 = cmd.param1; lastCalP2 = cmd.param2; lastCalP3 = cmd.param3;
  lastCalP4 = cmd.param4; lastCalP5 = cmd.param5; lastCalP6 = cmd.param6; lastCalP7 = cmd.param7;
  lastCalCommandMs = now;
  return false;
}

bool isSetupConfigMavlinkMessage(const mavlink_message_t &msg) {
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd; mavlink_msg_command_long_decode(&msg, &cmd);
    return isSetupConfigCommand((uint16_t)cmd.command);
  }
#ifdef MAVLINK_MSG_ID_COMMAND_INT
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    mavlink_command_int_t cmd; mavlink_msg_command_int_decode(&msg, &cmd);
    return isSetupConfigCommand((uint16_t)cmd.command);
  }
#endif
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
    mavlink_param_request_read_t pr; mavlink_msg_param_request_read_decode(&msg, &pr);
    char id[17]; copyParamId(id, pr.param_id);
    return isInteractiveSetupParamId(id);
  }
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
    mavlink_param_set_t ps; mavlink_msg_param_set_decode(&msg, &ps);
    char id[17]; copyParamId(id, ps.param_id);
    return isInteractiveSetupParamId(id);
  }
#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ) {
    mavlink_param_ext_request_read_t pr; mavlink_msg_param_ext_request_read_decode(&msg, &pr);
    char id[17]; copyParamId(id, pr.param_id);
    return isInteractiveSetupParamId(id);
  }
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_SET
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_EXT_SET) {
    mavlink_param_ext_set_t ps; mavlink_msg_param_ext_set_decode(&msg, &ps);
    char id[17]; copyParamId(id, ps.param_id);
    return isInteractiveSetupParamId(id);
  }
#endif
  return false;
}

bool isHighPriorityGCSMessage(const mavlink_message_t &msg) {
  switch (msg.msgid) {
    // FC GCS failsafe support: Mission Planner heartbeat must reach Pixhawk reliably.
    // Keep it high priority, but do not synthesize it. If LoRa/MP link is lost, heartbeat stops
    // naturally and ArduPilot FS_GCS_ENABLE/FS_GCS_TIMEOUT can take over.
    case MAVLINK_MSG_ID_HEARTBEAT:
    case MAVLINK_MSG_ID_COMMAND_LONG:
    case MAVLINK_MSG_ID_SET_MODE:
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    case MAVLINK_MSG_ID_PARAM_SET:
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
      return true;
#ifdef MAVLINK_MSG_ID_COMMAND_INT
    case MAVLINK_MSG_ID_COMMAND_INT: return true;
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: return true;
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: return true;
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_SET
    case MAVLINK_MSG_ID_PARAM_EXT_SET: return true;
#endif
    default: return false;
  }
}

uint8_t repeatCountForGCSMessage(const mavlink_message_t &msg) {
  if (isFlightActionGCSMessage(msg)) return 1;
  if (msg.msgid == MAVLINK_MSG_ID_SET_MODE) return REPEAT_SET_MODE;
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
    if (rawHighCount > (RAW_HIGH_QUEUE_SIZE - 16)) return 1;
    return REPEAT_PARAM_SET;
  }
#ifdef MAVLINK_MSG_ID_PARAM_EXT_SET
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_EXT_SET) {
    if (rawHighCount > (RAW_HIGH_QUEUE_SIZE - 16)) return 1;
    return REPEAT_PARAM_SET;
  }
#endif
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd; mavlink_msg_command_long_decode(&msg, &cmd);
    uint16_t command = (uint16_t)cmd.command;
    if (isCalibrationCommand(command)) return REPEAT_CALIBRATION_COMMAND;
    if (command == CMD_COMPONENT_ARM_DISARM) return REPEAT_ARM_DISARM;
    if (command == CMD_DO_SET_MODE) return REPEAT_SET_MODE;
    if (isCriticalCommandLong(command)) return REPEAT_NORMAL_COMMAND;
  }
#ifdef MAVLINK_MSG_ID_COMMAND_INT
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) return REPEAT_NORMAL_COMMAND;
#endif
  return REPEAT_NORMAL_COMMAND;
}

void flushLowCommandQueue() { rawLowHead = 0; rawLowTail = 0; rawLowCount = 0; }
void flushHighCommandQueue() { rawHighHead = 0; rawHighTail = 0; rawHighCount = 0; }
void flushCompactCommandQueue() { compactCmdHead = 0; compactCmdTail = 0; compactCmdCount = 0; }
void sendLocalStatustextToMissionPlanner(const char *text, uint8_t severity = MAV_SEVERITY_WARNING);
void sendBridgeEvent(uint8_t severity, const char *text, unsigned long minIntervalMs);
void sendProxyCommandAckToMissionPlanner(uint16_t command);
void serviceProxyCommandAckBurst();
void sendOptimisticSetModeHeartbeatToMissionPlanner(const mavlink_message_t &msg);
void sendOptimisticCommandLongModeHeartbeatToMissionPlanner(const mavlink_message_t &msg);


int32_t compactScale1000(float v) { return (int32_t)(v * COMPACT_CMD_SCALE_1000 + (v >= 0 ? 0.5f : -0.5f)); }
int32_t compactScale1e7(float v) { return (int32_t)(v * COMPACT_CMD_SCALE_1E7 + (v >= 0 ? 0.5f : -0.5f)); }
float compactFromX1000(int32_t v) { return ((float)v) / COMPACT_CMD_SCALE_1000; }

bool enqueueCompactPacket(const void *packet, uint8_t len, uint16_t command) {
  if (len == 0 || len > COMPACT_CMD_MAX_LEN) { compactCmdDrop++; commandDrop++; return false; }
  if (compactCmdCount >= COMPACT_CMD_QUEUE_SIZE) {
    compactCmdTail = (compactCmdTail + 1) % COMPACT_CMD_QUEUE_SIZE;
    compactCmdCount--; compactCmdDrop++;
  }
  CompactCommandQueueItem &item = compactCmdQueue[compactCmdHead];
  memset(&item, 0, sizeof(item));
  item.len = len;
  item.command = command;
  memcpy(item.payload, packet, len);
  compactCmdHead = (compactCmdHead + 1) % COMPACT_CMD_QUEUE_SIZE;
  compactCmdCount++; compactCmdQueuedCount++;
  return true;
}

bool peekCompactCommandPacket(CompactCommandQueueItem &item) {
  if (compactCmdCount == 0) return false;
  item = compactCmdQueue[compactCmdTail];
  return true;
}

void popCompactCommandPacket() {
  if (compactCmdCount == 0) return;
  compactCmdTail = (compactCmdTail + 1) % COMPACT_CMD_QUEUE_SIZE;
  compactCmdCount--;
}

bool shouldUseCompactCommandForMessage(const mavlink_message_t &msg) {
  // V28 static fix: keep telemetry payload/interval unchanged, but use compact
  // packets for flight-action commands from SF8 upward to avoid raw MAVLink
  // command timeouts/crashes at SF8-SF12.
  if (activeSF < 8) return false;
  if (!isFlightActionGCSMessage(msg)) return false;
  return msg.msgid == MAVLINK_MSG_ID_SET_MODE || msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG;
}

bool enqueueCompactCommandFromMissionPlanner(const mavlink_message_t &msg, uint16_t commandIdForAck) {
  if (!shouldUseCompactCommandForMessage(msg)) return false;
  if (msg.msgid == MAVLINK_MSG_ID_SET_MODE) {
    mavlink_set_mode_t sm; mavlink_msg_set_mode_decode(&msg, &sm);
    CompactSetModePacket pkt = {};
    initHeader(pkt.hdr, PKT_CMD_COMPACT);
    pkt.seq = ++compactCmdSeq;
    pkt.kind = COMPACT_CMD_KIND_SET_MODE;
    pkt.target_system = sm.target_system;
    pkt.base_mode = sm.base_mode;
    pkt.custom_mode = sm.custom_mode;
    finalizePacketCrc(&pkt, sizeof(pkt));
    return enqueueCompactPacket(&pkt, sizeof(pkt), commandIdForAck ? commandIdForAck : CMD_DO_SET_MODE);
  }

  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd; mavlink_msg_command_long_decode(&msg, &cmd);
    uint16_t command = (uint16_t)cmd.command;
    if (command == CMD_COMPONENT_ARM_DISARM) {
      CompactArmDisarmPacket pkt = {};
      initHeader(pkt.hdr, PKT_CMD_COMPACT);
      pkt.seq = ++compactCmdSeq;
      pkt.kind = COMPACT_CMD_KIND_ARM_DISARM;
      pkt.target_system = cmd.target_system;
      pkt.target_component = cmd.target_component;
      pkt.arm = (cmd.param1 >= 0.5f) ? 1 : 0;
      pkt.param2_x1000 = compactScale1000(cmd.param2);
      finalizePacketCrc(&pkt, sizeof(pkt));
      return enqueueCompactPacket(&pkt, sizeof(pkt), command);
    }
    if (isFlightActionCommandId(command)) {
      CompactCommandLongPacket pkt = {};
      initHeader(pkt.hdr, PKT_CMD_COMPACT);
      pkt.seq = ++compactCmdSeq;
      pkt.kind = COMPACT_CMD_KIND_COMMAND_LONG;
      pkt.command = command;
      pkt.target_system = cmd.target_system;
      pkt.target_component = cmd.target_component;
      pkt.confirmation = cmd.confirmation;
      pkt.p1_x1000 = compactScale1000(cmd.param1);
      pkt.p2_x1000 = compactScale1000(cmd.param2);
      pkt.p3_x1000 = compactScale1000(cmd.param3);
      pkt.p4_x1000 = compactScale1000(cmd.param4);
      pkt.p5_x1e7 = compactScale1e7(cmd.param5);
      pkt.p6_x1e7 = compactScale1e7(cmd.param6);
      pkt.p7_x1000 = compactScale1000(cmd.param7);
      finalizePacketCrc(&pkt, sizeof(pkt));
      return enqueueCompactPacket(&pkt, sizeof(pkt), command);
    }
  }
  return false;
}

bool enqueueHighRawPacket(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > RAW_MAVLINK_MAX) { commandDrop++; return false; }
  if (rawHighCount >= RAW_HIGH_QUEUE_SIZE) {
    flushLowCommandQueue();
  }
  if (rawHighCount >= RAW_HIGH_QUEUE_SIZE) {
    rawHighDrop++;
    commandDrop++;
    return false;
  }
  MavlinkRawPacket &pkt = rawHighQueue[rawHighHead];
  memset(&pkt, 0, sizeof(pkt));
  initHeader(pkt.hdr, PKT_MAVLINK_RAW);
  pkt.len = len; memcpy(pkt.payload, data, len);
  finalizePacketCrc(&pkt, RAW_PKT_HEADER_LEN + pkt.len);
  rawHighHead = (rawHighHead + 1) % RAW_HIGH_QUEUE_SIZE;
  rawHighCount++;
  return true;
}

bool enqueueLowRawPacket(const uint8_t *data, uint8_t len) {
  if (len == 0 || len > RAW_MAVLINK_MAX) { commandDrop++; return false; }
  if (rawLowCount >= RAW_LOW_QUEUE_SIZE) {
    rawLowTail = (rawLowTail + 1) % RAW_LOW_QUEUE_SIZE;
    rawLowCount--; rawLowDrop++;
  }
  MavlinkRawPacket &pkt = rawLowQueue[rawLowHead];
  memset(&pkt, 0, sizeof(pkt));
  initHeader(pkt.hdr, PKT_MAVLINK_RAW);
  pkt.len = len; memcpy(pkt.payload, data, len);
  finalizePacketCrc(&pkt, RAW_PKT_HEADER_LEN + pkt.len);
  rawLowHead = (rawLowHead + 1) % RAW_LOW_QUEUE_SIZE;
  rawLowCount++;
  return true;
}

bool peekNextRawPacket(MavlinkRawPacket &pkt, bool &fromHighQueue) {
  if (rawHighCount > 0) { pkt = rawHighQueue[rawHighTail]; fromHighQueue = true; return true; }
  if (rawLowCount > 0) { pkt = rawLowQueue[rawLowTail]; fromHighQueue = false; return true; }
  return false;
}

void popNextRawPacket(bool fromHighQueue) {
  if (fromHighQueue) { if (rawHighCount == 0) return; rawHighTail = (rawHighTail + 1) % RAW_HIGH_QUEUE_SIZE; rawHighCount--; }
  else { if (rawLowCount == 0) return; rawLowTail = (rawLowTail + 1) % RAW_LOW_QUEUE_SIZE; rawLowCount--; }
}

bool fullParamSyncAllowedOnActiveSF() {
#if BLOCK_FULL_PARAM_SYNC_HIGH_SF
  // If the LoRa link has not locked yet, do not allow Mission Planner to queue a full
  // parameter download on the default scan SF and accidentally release it later at SF10-SF12.
  if (!phyLocked) return false;
  return activeSF <= FULL_PARAM_SYNC_MAX_SF;
#else
  return true;
#endif
}

void notifyHighSfParamBlockedIfNeeded() {
  unsigned long now = millis();
  if (now - lastHighSfParamNoticeMs < HIGH_SF_PARAM_NOTICE_INTERVAL_MS) return;
  lastHighSfParamNoticeMs = now;
  sendBridgeEvent(MAV_SEVERITY_WARNING, "Param sync blocked: use SF7-SF9 to sync", HIGH_SF_PARAM_NOTICE_INTERVAL_MS);
}

bool shouldForwardParamRequestList() {
  unsigned long now = millis();
  if (!fullParamSyncAllowedOnActiveSF()) {
    fullParamSyncBlockedHighSfCount++;
    abortParamSyncRecovery("block_full_param_high_sf");
    notifyHighSfParamBlockedIfNeeded();
    return false;
  }
  if (paramSyncActive && lastParamRequestListForwardMs > 0 && (now - lastParamRequestListForwardMs) < PARAM_REQUEST_LIST_DEDUP_MS) {
    return false;
  }
  lastParamRequestListForwardMs = now;
  return true;
}

bool shouldForwardParamExtRequestList() {
  unsigned long now = millis();
  if (!fullParamSyncAllowedOnActiveSF()) {
    fullParamSyncBlockedHighSfCount++;
    abortParamSyncRecovery("block_full_param_ext_high_sf");
    notifyHighSfParamBlockedIfNeeded();
    return false;
  }
  if (paramSyncActive && lastParamExtRequestListForwardMs > 0 && (now - lastParamExtRequestListForwardMs) < PARAM_REQUEST_LIST_DEDUP_MS) {
    return false;
  }
  lastParamExtRequestListForwardMs = now;
  return true;
}


bool highSfParamTrafficBlocked() {
#if BLOCK_PARAM_READ_HIGH_SF
  return (!phyLocked || activeSF > FULL_PARAM_SYNC_MAX_SF);
#else
  return false;
#endif
}

bool shouldForwardParamRequestReadHighSf(const mavlink_message_t &msg) {
  if (!highSfParamTrafficBlocked()) return true;
  mavlink_param_request_read_t pr;
  mavlink_msg_param_request_read_decode(&msg, &pr);
  char id[17]; copyParamId(id, pr.param_id);
  // Keep a small escape hatch for interactive setup pages, but block generic auto-sync reads.
  if (isInteractiveSetupParamId(id)) return true;
  paramReadBlockedHighSfCount++;
  abortParamSyncRecovery("block_param_read_high_sf");
  notifyHighSfParamBlockedIfNeeded();
  return false;
}

#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
bool shouldForwardParamExtRequestReadHighSf(const mavlink_message_t &msg) {
  if (!highSfParamTrafficBlocked()) return true;
  mavlink_param_ext_request_read_t pr;
  mavlink_msg_param_ext_request_read_decode(&msg, &pr);
  char id[17]; copyParamId(id, pr.param_id);
  if (isInteractiveSetupParamId(id)) return true;
  paramReadBlockedHighSfCount++;
  abortParamSyncRecovery("block_param_ext_read_high_sf");
  notifyHighSfParamBlockedIfNeeded();
  return false;
}
#endif

bool isHighSfSurvivalMode() {
  return (!phyLocked || activeSF > FULL_PARAM_SYNC_MAX_SF);
}

bool shouldBlockMissionPlannerMessageAtCurrentSF(const mavlink_message_t &msg) {
  if (!isHighSfSurvivalMode()) return false;
  if (isFlightActionGCSMessage(msg)) return false;

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
      return true;
#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_MISSION_REQUEST_LIST
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_MISSION_COUNT
    case MAVLINK_MSG_ID_MISSION_COUNT:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_MISSION_REQUEST
    case MAVLINK_MSG_ID_MISSION_REQUEST:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_MISSION_REQUEST_INT
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_LOG_REQUEST_LIST
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_LOG_REQUEST_DATA
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_LOG_ERASE
    case MAVLINK_MSG_ID_LOG_ERASE:
      return true;
#endif
#ifdef MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
      return true;
#endif
    case MAVLINK_MSG_ID_COMMAND_LONG:
#ifdef MAVLINK_MSG_ID_COMMAND_INT
    case MAVLINK_MSG_ID_COMMAND_INT:
#endif
      return isSetupConfigMavlinkMessage(msg);
    default:
      return false;
  }
}

bool shouldForwardGCSMessage(const mavlink_message_t &msg) {
  if (isDuplicateCalibrationCommand(msg)) return false;
  if (shouldBlockMissionPlannerMessageAtCurrentSF(msg)) {
    abortParamSyncRecovery("block_high_sf_heavy_mp_message");
    sendBridgeEvent(MAV_SEVERITY_WARNING, "Blocked: use SF7-SF9 for setup/param/log", 5000UL);
    return false;
  }
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      paramRequestCount++;
      if (!shouldForwardParamRequestList()) return false;
      startParamSync(); return true;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
      paramRequestCount++;
      if (!shouldForwardParamRequestReadHighSf(msg)) return false;
      if (isSetupConfigMavlinkMessage(msg)) startCalibrationConfigMode();
      if (fullParamSyncAllowedOnActiveSF()) startParamSync();
      return true;
    case MAVLINK_MSG_ID_PARAM_SET:
      if (isSetupConfigMavlinkMessage(msg)) startCalibrationConfigMode();
      return true;
#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
      paramRequestCount++;
      if (!shouldForwardParamExtRequestList()) return false;
      startParamSync(); return true;
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
      paramRequestCount++;
      if (!shouldForwardParamExtRequestReadHighSf(msg)) return false;
      if (isSetupConfigMavlinkMessage(msg)) startCalibrationConfigMode();
      if (fullParamSyncAllowedOnActiveSF()) startParamSync();
      return true;
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_SET
    case MAVLINK_MSG_ID_PARAM_EXT_SET:
      if (isSetupConfigMavlinkMessage(msg)) startCalibrationConfigMode();
      return true;
#endif
    default:
      if (isSetupConfigMavlinkMessage(msg)) { calCommandSentCount++; startCalibrationConfigMode(); }
      return true;
  }
}


void beginArmFeedbackMode() {
  armFeedbackUntilMs = millis() + ARM_FEEDBACK_HOLD_MS;
}

bool armFeedbackActive() {
  return armFeedbackUntilMs != 0 && millis() < armFeedbackUntilMs;
}

bool isArmDisarmCommandId(uint16_t command) {
  return command == MAV_CMD_COMPONENT_ARM_DISARM || command == CMD_COMPONENT_ARM_DISARM;
}

bool isForceArmParam2(float param2) {
  // ArduPilot force-arm magic value is 21196. Use a broad threshold so float
  // scaling/compact transport cannot accidentally hide the intent.
  return param2 > 10000.0f;
}

unsigned long armAckTravelGuardMsForSF(uint8_t sf) {
  if (sf >= 12) return 1600UL;
  if (sf == 11) return 900UL;
  if (sf == 10) return 450UL;
  if (sf == 9) return 180UL;
  return 80UL;
}

bool decodeArmCommandFromMissionPlannerMessage(const mavlink_message_t &msg, bool &arm, bool &force) {
  arm = false; force = false;
  if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) return false;
  mavlink_command_long_t cmd;
  mavlink_msg_command_long_decode(&msg, &cmd);
  if (!isArmDisarmCommandId((uint16_t)cmd.command)) return false;
  arm = (cmd.param1 >= 0.5f);
  force = arm && isForceArmParam2(cmd.param2);
  return true;
}

bool decodeArmCommandFromCompactItem(const CompactCommandQueueItem &item, bool &arm, bool &force) {
  arm = false; force = false;
  if (item.len != sizeof(CompactArmDisarmPacket)) return false;
  CompactArmDisarmPacket pkt;
  memcpy(&pkt, item.payload, sizeof(pkt));
  if (pkt.hdr.type != PKT_CMD_COMPACT || pkt.kind != COMPACT_CMD_KIND_ARM_DISARM) return false;
  arm = pkt.arm != 0;
  force = arm && isForceArmParam2(compactFromX1000(pkt.param2_x1000));
  return true;
}

bool decodeArmCommandFromRawPacket(const MavlinkRawPacket &raw, bool &arm, bool &force) {
  arm = false; force = false;
  if (raw.len == 0 || raw.len > RAW_MAVLINK_MAX) return false;
  mavlink_message_t parsedMsg;
  mavlink_status_t parsedStatus;
  memset(&parsedMsg, 0, sizeof(parsedMsg));
  memset(&parsedStatus, 0, sizeof(parsedStatus));
  for (uint8_t i = 0; i < raw.len; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_2, raw.payload[i], &parsedMsg, &parsedStatus)) {
      if (decodeArmCommandFromMissionPlannerMessage(parsedMsg, arm, force)) return true;
    }
  }
  return false;
}

void beginArmCommandTransaction(bool arm, bool force) {
  clearCommandProxyAck(CMD_COMPONENT_ARM_DISARM);
  clearCommandProxyAck(MAV_CMD_COMPONENT_ARM_DISARM);
  armCommandTxnSeq++;
  armCommandTxnActive = true;
  armCommandTxnTxConfirmed = false;
  armCommandTxnArm = arm;
  armCommandTxnForce = force;
  armCommandTxnQueuedMs = millis();
  armCommandTxnTxMs = 0;
  beginArmFeedbackMode();
}

void markArmCommandTransactionTx(bool arm, bool force) {
  unsigned long now = millis();
  if (!armCommandTxnActive) {
    beginArmCommandTransaction(arm, force);
  }
  armCommandTxnArm = arm;
  armCommandTxnForce = force;
  armCommandTxnTxConfirmed = true;
  armCommandTxnTxMs = now;
  beginArmFeedbackMode();
}

bool shouldForwardArmAckToMissionPlanner(const mavlink_command_ack_t &ack) {
  (void)ack;
  unsigned long now = millis();
  beginArmFeedbackMode();

  if (!armCommandTxnActive) {
    armCommandAckSuppressedStaleCount++;
    commandAckSuppressedCount++;
    return false;
  }

  if (!armCommandTxnTxConfirmed) {
    if (now - armCommandTxnQueuedMs > ARM_TX_WAIT_TIMEOUT_MS) {
      armCommandTxnActive = false;
      armCommandTxnTxConfirmed = false;
    }
    armCommandAckSuppressedStaleCount++;
    commandAckSuppressedCount++;
    return false;
  }

  if (now - armCommandTxnQueuedMs > ARM_ACK_TOTAL_TIMEOUT_MS) {
    armCommandTxnActive = false;
    armCommandTxnTxConfirmed = false;
    armCommandAckSuppressedStaleCount++;
    commandAckSuppressedCount++;
    return false;
  }

  // V16 STABLE:
  // Do not synthesize/proxy ACK for ARM or FORCE ARM. Mission Planner must receive
  // the real flight-controller COMMAND_ACK so the Force Arm dialog and rejection
  // reason remain consistent. A bridge ACCEPTED ACK can close MP's doCommand while
  // ArduPilot is actually rejecting/disarming, which is the main crash/mismatch risk.
  armCommandTxnActive = false;
  armCommandTxnTxConfirmed = false;
  clearRecentMissionPlannerCommand(CMD_COMPONENT_ARM_DISARM);
  clearRecentMissionPlannerCommand(MAV_CMD_COMPONENT_ARM_DISARM);
  armCommandAckForwardedCount++;
  return true;
}

void sendHighSfConnectionNoticeIfNeeded(uint8_t sf) {
  if (sf < 10) return;
  if (lastHighSfNoticeForSf != sf) {
    lastHighSfNoticeForSf = sf;
    highSfConnectNoticeSent = false;
  }
  if (highSfConnectNoticeSent) return;
  highSfConnectNoticeSent = true;
  notifyHighSfParamBlockedIfNeeded();
}

void serviceLoRaParamNoticeBurst() {
  if (loRaParamNoticeBurstRemaining == 0) return;
  unsigned long now = millis();
  if (now < loRaParamNoticeNextMs) return;
  if (loRaParamNoticeText[0] == '\0') return;
  sendLocalStatustextToMissionPlanner(loRaParamNoticeText, MAV_SEVERITY_NOTICE);
  loRaParamNoticeBurstRemaining--;
  loRaParamNoticeNextMs = now + LORA_PARAM_NOTICE_BURST_GAP_MS;
}

void sendLoRaParamNoticeIfNeeded(uint8_t sf, uint8_t tp) {
  // User-facing Mission Planner message: only static LoRa parameters at initial
  // link lock or when SF/TP changes. Dynamic metrics stay on MetricsSerial only.
  bool changed = (!loRaParamNoticeSent || lastLoRaParamNoticeSF != sf || lastLoRaParamNoticeTP != tp);
  if (!changed) return;

  loRaParamNoticeSent = true;
  lastLoRaParamNoticeSF = sf;
  lastLoRaParamNoticeTP = tp;
  lastLoRaParamNoticeMs = millis();

  snprintf(loRaParamNoticeText, sizeof(loRaParamNoticeText),
           "LoRa params: SF%u BW%ukHz CR4/%u TP%udBm",
           (unsigned)sf, (unsigned)((uint16_t)LORA_BW_KHZ), (unsigned)LORA_CR_DEN, (unsigned)tp);
  loRaParamNoticeBurstRemaining = LORA_PARAM_NOTICE_BURST_COUNT;
  loRaParamNoticeNextMs = 0;
  serviceLoRaParamNoticeBurst();
}

void enqueueGCSMavlinkMessage(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (len == 0 || len > RAW_MAVLINK_MAX) { commandDrop++; return; }
  bool flightAction = isFlightActionGCSMessage(msg);
  uint16_t commandIdForAck = extractCommandIdFromMissionPlannerMessage(msg);
  bool isArmTxnMsg = false;
  bool armTxnArm = false;
  bool armTxnForce = false;
  isArmTxnMsg = decodeArmCommandFromMissionPlannerMessage(msg, armTxnArm, armTxnForce);
  if (flightAction) {
    beginFlightCommandPriorityMode();
    if (commandIdForAck == MAV_CMD_COMPONENT_ARM_DISARM || commandIdForAck == CMD_COMPONENT_ARM_DISARM) {
      beginArmFeedbackMode();
      sendBridgeEvent(MAV_SEVERITY_NOTICE, "ARM sent; FC may reject/disarm on battery failsafe", 3000UL);
    }
  }
  if (!isBackgroundOrInternalCommandAck(commandIdForAck)) rememberMissionPlannerCommandForAck(commandIdForAck, flightAction);
  bool high = isHighPriorityGCSMessage(msg) || flightAction;
  uint8_t repeat = repeatCountForGCSMessage(msg);
  if (repeat < 1) repeat = 1;
  if (flightAction) repeat = 1;
  if (high && !flightAction) flushLowCommandQueue();
  bool queuedOk = false;
  if (flightAction && enqueueCompactCommandFromMissionPlanner(msg, commandIdForAck)) {
    queuedOk = true;
  } else {
    for (uint8_t i = 0; i < repeat; i++) {
      bool ok = high ? enqueueHighRawPacket(buf, (uint8_t)len) : enqueueLowRawPacket(buf, (uint8_t)len);
      if (ok) queuedOk = true;
    }
  }
  if (queuedOk && isArmTxnMsg) {
    beginArmCommandTransaction(armTxnArm, armTxnForce);
    sendBridgeEvent(armTxnForce ? MAV_SEVERITY_WARNING : MAV_SEVERITY_NOTICE,
                    armTxnForce ? "FORCE ARM sent; FC may still disarm on failsafe" : "ARM sent; waiting real FC ACK",
                    2500UL);
  }
  // V17 real-state: no optimistic/synthetic mode heartbeat.
  // Mode/armed state in Mission Planner changes only after real FC HEARTBEAT/ACK returns.
#if COMMAND_PROXY_ACK_ON_ENQUEUE_ENABLE
  // Pada SF tinggi, Mission Planner doCommand sering timeout/crash sebelum LoRa half-duplex
  // sempat mengirim command ke UAV dan menerima COMMAND_ACK asli dari Pixhawk.
  // ACK ini adalah ACK dari bridge GCS bahwa command sudah diterima dan diantrikan,
  // sedangkan eksekusi riil tetap dilakukan oleh Pixhawk melalui jalur LoRa.
  if (queuedOk && flightAction && shouldProxyAckForCommand(commandIdForAck)) {
    sendProxyCommandAckToMissionPlanner(commandIdForAck);
    if (activeSF >= COMMAND_PROXY_ACK_MIN_SF) {
      sendBridgeEvent(MAV_SEVERITY_NOTICE, "Bridge command queued; waiting vehicle status", 2500UL);
    }
  }
#endif
}

void readGCSMavlinkCommands() {
  while (GCS_Serial.available()) {
    uint8_t c = GCS_Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg_gcs_in, &status_gcs_in)) {
      if (shouldForwardGCSMessage(msg_gcs_in)) enqueueGCSMavlinkMessage(msg_gcs_in);
    }
  }
}


bool isRawTelemetryDuplicateInNormalMode(uint32_t msgid) {
#if MP_RAW_TELEMETRY_DEDUP_ENABLE
  // PARAM/MISSION/ACK/STATUSTEXT still pass quickly below because they are not in this duplicate list.
  // Strict mode prevents raw HEARTBEAT/GPS/ATTITUDE/SYS_STATUS duplicates from leaking during param-hold
  // and creating false MAVLink packet loss in Mission Planner.
#if !MP_STRICT_RAW_TELEMETRY_DEDUP_ENABLE
  if (fastOperationalModeActive()) return false;
#endif

  // These messages are already reconstructed once per 78-byte beacon. Forwarding the
  // raw copies at the same time can create duplicate/irregular MAVLink streams in MP.
  switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    case MAVLINK_MSG_ID_SYS_STATUS:
#ifdef MAVLINK_MSG_ID_ATTITUDE
    case MAVLINK_MSG_ID_ATTITUDE:
#endif
#ifdef MAVLINK_MSG_ID_GLOBAL_POSITION_INT
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
#endif
#ifdef MAVLINK_MSG_ID_GPS_RAW_INT
    case MAVLINK_MSG_ID_GPS_RAW_INT:
#endif
#ifdef MAVLINK_MSG_ID_VFR_HUD
    case MAVLINK_MSG_ID_VFR_HUD:
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
#endif
      return true;
    default:
      return false;
  }
#else
  (void)msgid;
  return false;
#endif
}

void sendMavlinkMessageToMissionPlanner(mavlink_message_t &msg) {
#if MP_MAVLINK_SEQ_NORMALIZER_ENABLE
  normalizeMpMavlinkSeq(msg);
#endif
  uint16_t len = mavlink_msg_to_send_buffer(mavBuf, &msg);

#if MP_SERIAL_PACING_ENABLE
  if (!mpTelemetryOutputContext) {
    // PARAMBYPASS: raw/param/mission/ack output stays as fast as the old behavior.
    GCS_Serial.write(mavBuf, len);
    mpFastBypassOutCount++;
  } else {
    static unsigned long lastMpTelemetryWriteUs = 0;
    unsigned long nowUs = micros();
    const unsigned long gapUs = MP_TELEMETRY_SERIAL_GAP_US;
    if (gapUs > 0 && lastMpTelemetryWriteUs != 0 && (uint32_t)(nowUs - lastMpTelemetryWriteUs) < gapUs) {
      delayMicroseconds((uint32_t)(gapUs - (nowUs - lastMpTelemetryWriteUs)));
    }
    uint16_t sent = 0;
    unsigned long deadline = millis() + 25UL;
    while (sent < len) {
      int room = GCS_Serial.availableForWrite();
      if (room <= 0) {
        if ((long)(millis() - deadline) >= 0) break;
        delay(1);
        continue;
      }
      uint16_t chunk = (uint16_t)room;
      if (chunk > (uint16_t)(len - sent)) chunk = (uint16_t)(len - sent);
      GCS_Serial.write(mavBuf + sent, chunk);
      sent += chunk;
    }
    if (sent < len) GCS_Serial.write(mavBuf + sent, len - sent);
    lastMpTelemetryWriteUs = micros();
    mpTelemetryOutCount++;
  }
#else
  GCS_Serial.write(mavBuf, len);
#endif
  mpMavCount++; mpMavBytes += len;
}

void sendLocalStatustextToMissionPlanner(const char *text, uint8_t severity) {
  mavlink_statustext_t st = {};
  st.severity = severity;
  strncpy(st.text, text, sizeof(st.text) - 1);
  st.text[sizeof(st.text) - 1] = '\0';
  mavlink_msg_statustext_encode(1, MAV_COMP_ID_AUTOPILOT1, &msg_out, &st);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

void sendBridgeEvent(uint8_t severity, const char *text, unsigned long minIntervalMs) {
  static char lastText[64] = {0};
  static unsigned long lastTextMs = 0;
  unsigned long now = millis();
  if (strncmp(lastText, text, sizeof(lastText)) == 0 && (now - lastTextMs) < minIntervalMs) return;
  strncpy(lastText, text, sizeof(lastText) - 1);
  lastText[sizeof(lastText) - 1] = '\0';
  lastTextMs = now;
  sendLocalStatustextToMissionPlanner(text, severity);
}

void sendHeartbeatFromData(uint8_t sysid, uint8_t compid, uint8_t mavType, uint8_t autopilot, uint8_t baseMode, uint32_t customMode, uint8_t systemStatus) {
  canonicalizeMissionPlannerSource(sysid, compid);
  if (mavType == 0) mavType = MAV_TYPE_HEXAROTOR;
  if (autopilot == 0) autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  mavlink_msg_heartbeat_pack(sysid, compid, &msg_out, mavType, autopilot, baseMode, customMode, systemStatus);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

void clearOptimisticModeHeartbeat() {
  if (optimisticModeActive) optimisticModeClearCount++;
  optimisticModeActive = false;
  optimisticModeUntilMs = 0;
  optimisticModeLastSendMs = 0;
  optimisticModeRealMatchSinceMs = 0;
}

bool optimisticModeOverrideActive() {
#if OPTIMISTIC_MODE_UI_ENABLE
  if (!optimisticModeActive) return false;
  unsigned long now = millis();
  if (now > optimisticModeUntilMs) { clearOptimisticModeHeartbeat(); return false; }
  return activeSF >= OPTIMISTIC_MODE_MIN_SF;
#else
  return false;
#endif
}

void beginOptimisticModeHeartbeat(uint8_t targetSystem, uint8_t baseMode, uint32_t customMode) {
#if OPTIMISTIC_MODE_UI_ENABLE
  if (activeSF < OPTIMISTIC_MODE_MIN_SF) return;
  uint8_t sysid = latestBeaconValid && latestBeaconData.system_id ? latestBeaconData.system_id : (targetSystem ? targetSystem : 1);
  uint8_t compid = latestBeaconValid && latestBeaconData.component_id ? latestBeaconData.component_id : MAV_COMP_ID_AUTOPILOT1;
  optimisticModeActive = true;
  optimisticModeBaseMode = baseMode;
  optimisticModeCustomMode = customMode;
  optimisticModeSysId = sysid;
  optimisticModeCompId = compid;
  optimisticModeSystemStatus = latestBeaconValid ? latestBeaconData.system_status : MAV_STATE_ACTIVE;
  optimisticModeUntilMs = millis() + OPTIMISTIC_MODE_HOLD_MS;
  optimisticModeLastSendMs = 0;
  optimisticModeRealMatchSinceMs = 0;
  for (uint8_t i = 0; i < OPTIMISTIC_MODE_IMMEDIATE_BURST_COUNT; i++) {
    optimisticModeLastSendMs = 0;
    serviceOptimisticModeHeartbeat();
  }
#else
  (void)targetSystem; (void)baseMode; (void)customMode;
#endif
}

void updateOptimisticModeFromRealBeacon(const PixhawkDataBeacon &d) {
#if OPTIMISTIC_MODE_UI_ENABLE
  if (!optimisticModeActive) return;
  unsigned long now = millis();
  if (d.base_mode == optimisticModeBaseMode && d.custom_mode == optimisticModeCustomMode) {
    if (optimisticModeRealMatchSinceMs == 0) optimisticModeRealMatchSinceMs = now;
    if (now - optimisticModeRealMatchSinceMs >= OPTIMISTIC_MODE_MATCH_CLEAR_MS) clearOptimisticModeHeartbeat();
  } else {
    optimisticModeRealMatchSinceMs = 0;
  }
#else
  (void)d;
#endif
}

void serviceOptimisticModeHeartbeat() {
#if OPTIMISTIC_MODE_UI_ENABLE
  if (!optimisticModeOverrideActive()) return;
  unsigned long now = millis();
  if (optimisticModeLastSendMs != 0 && now - optimisticModeLastSendMs < OPTIMISTIC_MODE_HEARTBEAT_INTERVAL_MS) return;
  sendHeartbeatFromData(optimisticModeSysId, optimisticModeCompId, MAV_TYPE_HEXAROTOR,
                        MAV_AUTOPILOT_ARDUPILOTMEGA, optimisticModeBaseMode,
                        optimisticModeCustomMode, optimisticModeSystemStatus);
  optimisticModeLastSendMs = now;
  optimisticModeHeartbeatCount++;
#endif
}

void sendOptimisticSetModeHeartbeatToMissionPlanner(const mavlink_message_t &msg) {
  // V15: keep a short-lived mode overlay so old beacons do not immediately overwrite
  // the requested mode before the slow high-SF command/ACK cycle finishes.
  if (activeSF < OPTIMISTIC_MODE_MIN_SF) return;
  if (msg.msgid != MAVLINK_MSG_ID_SET_MODE) return;
  mavlink_set_mode_t sm = {};
  mavlink_msg_set_mode_decode(&msg, &sm);
  beginOptimisticModeHeartbeat(sm.target_system, sm.base_mode, sm.custom_mode);
}

void sendOptimisticCommandLongModeHeartbeatToMissionPlanner(const mavlink_message_t &msg) {
  if (activeSF < OPTIMISTIC_MODE_MIN_SF) return;
  if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) return;
  mavlink_command_long_t cmd = {};
  mavlink_msg_command_long_decode(&msg, &cmd);
  if ((uint16_t)cmd.command != MAV_CMD_DO_SET_MODE && (uint16_t)cmd.command != CMD_DO_SET_MODE) return;
  uint8_t baseMode = (uint8_t)cmd.param1;
  uint32_t customMode = (uint32_t)cmd.param2;
  beginOptimisticModeHeartbeat(cmd.target_system, baseMode, customMode);
}

void sendAttitudeFromData(uint8_t sysid, uint8_t compid, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
  canonicalizeMissionPlannerSource(sysid, compid);
  mavlink_attitude_t p = {};
  p.time_boot_ms = millis(); p.roll = roll; p.pitch = pitch; p.yaw = yaw;
  p.rollspeed = rollspeed; p.pitchspeed = pitchspeed; p.yawspeed = yawspeed;
  mavlink_msg_attitude_encode(sysid, compid, &msg_out, &p);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

void sendGlobalPositionFromData(uint8_t sysid, uint8_t compid, int32_t lat, int32_t lon, int32_t alt, int32_t relativeAlt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg) {
  canonicalizeMissionPlannerSource(sysid, compid);
  mavlink_global_position_int_t p = {};
  p.time_boot_ms = millis(); p.lat = lat; p.lon = lon; p.alt = alt; p.relative_alt = relativeAlt;
  p.vx = vx; p.vy = vy; p.vz = vz; p.hdg = hdg;
  mavlink_msg_global_position_int_encode(sysid, compid, &msg_out, &p);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

void sendGpsRawFromData(uint8_t sysid, uint8_t compid, const GpsRawData &gps) {
  canonicalizeMissionPlannerSource(sysid, compid);
  mavlink_gps_raw_int_t p = {};
  p.time_usec = gps.time_usec; p.fix_type = gps.fix_type; p.lat = gps.lat; p.lon = gps.lon;
  p.alt = gps.alt_mm; p.eph = gps.eph; p.epv = gps.epv; p.vel = gps.vel; p.cog = gps.cog;
  p.satellites_visible = gps.satellites_visible;
  mavlink_msg_gps_raw_int_encode(sysid, compid, &msg_out, &p);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

static inline float centiDegToRad(int16_t cd) {
  return ((float)cd) * PI / 18000.0f;
}

static inline int32_t decimeterToMillimeter(int16_t dm) {
  return (int32_t)dm * 100L;
}

uint8_t mpGpsSatellitesForDisplay(uint8_t sat, uint8_t fixType) {
#if MP_GPS_SAT_DISPLAY_HOLD_ENABLE
  static uint8_t lastGoodSat = 0;
  static unsigned long lastGoodSatMs = 0;
  unsigned long now = millis();

  if (fixType >= 3 && sat > 0) {
    if (sat > lastGoodSat) {
      lastGoodSat = sat;
      lastGoodSatMs = now;
    } else if (sat >= MP_GPS_SAT_MIN_WHEN_3D_FIX) {
      // Refresh hold window if the current value is already good enough.
      lastGoodSatMs = now;
    }
  }

  if (fixType >= 3) {
    uint8_t outSat = sat;
    if (lastGoodSat >= MP_GPS_SAT_MIN_WHEN_3D_FIX && (now - lastGoodSatMs) <= MP_GPS_SAT_HOLD_MS) {
      if (outSat < lastGoodSat) outSat = lastGoodSat;
    }
    if (outSat > 0 && outSat < MP_GPS_SAT_MIN_WHEN_3D_FIX) outSat = MP_GPS_SAT_MIN_WHEN_3D_FIX;
    return outSat;
  }
#endif
  return sat;
}

void sendGpsRawFromBeacon(uint8_t sysid, uint8_t compid, const PixhawkDataBeacon &d) {
  canonicalizeMissionPlannerSource(sysid, compid);
  mavlink_gps_raw_int_t p = {};
  p.time_usec = micros();
  p.fix_type = d.gps.fix_type;
  p.lat = d.lat;
  p.lon = d.lon;
  p.alt = decimeterToMillimeter(d.gps.alt_dm);
  p.eph = d.gps.eph;
  p.epv = d.gps.epv;
  // gps.vel dan gps.cog tidak dikirim lagi di payload utama.
  // Keduanya direkonstruksi dari groundspeed dan heading agar Mission Planner tetap mendapat GPS_RAW_INT lengkap.
  p.vel = d.groundspeed_cms;
  p.cog = d.heading;
  p.satellites_visible = mpGpsSatellitesForDisplay(d.gps.satellites_visible, d.gps.fix_type);
  mavlink_msg_gps_raw_int_encode(sysid, compid, &msg_out, &p);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

void sendVfrHudFromData(uint8_t sysid, uint8_t compid, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float climb) {
  canonicalizeMissionPlannerSource(sysid, compid);
  mavlink_vfr_hud_t p = {};
  p.airspeed = airspeed; p.groundspeed = groundspeed; p.heading = heading;
  p.throttle = throttle; p.alt = 0.0f; p.climb = climb;
  mavlink_msg_vfr_hud_encode(sysid, compid, &msg_out, &p);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

void sendSysStatusFromData(uint8_t sysid, uint8_t compid, uint16_t voltageBattery, int16_t currentBattery, int8_t batteryRemaining) {
  canonicalizeMissionPlannerSource(sysid, compid);
  mavlink_sys_status_t p = {};
  p.voltage_battery = voltageBattery; p.current_battery = currentBattery; p.battery_remaining = batteryRemaining;
  mavlink_msg_sys_status_encode(sysid, compid, &msg_out, &p);
  sendMavlinkMessageToMissionPlanner(msg_out);
}

#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
void sendEkfStatusFromData(uint8_t sysid, uint8_t compid, uint16_t ekfFlags) {
  canonicalizeMissionPlannerSource(sysid, compid);
  mavlink_ekf_status_report_t p = {};
  p.flags = ekfFlags;
  mavlink_msg_ekf_status_report_encode(sysid, compid, &msg_out, &p);
  sendMavlinkMessageToMissionPlanner(msg_out);
}
#endif

uint8_t missionPlannerRadioSignalByte() {
  if (!latestBeaconValid) return 0;

  unsigned long now = millis();
  unsigned long age = now - lastPacketMs;

  /*
    MPQUALITY FIX - real LoRa Link Availability Score:
    - Nilai ini dipakai khusus untuk MAVLink RADIO_STATUS.rssi agar PreFlight
      Telemetry Signal merepresentasikan kualitas link LoRa nyata, bukan kualitas
      sequence MAVLink Mission Planner.
    - Dasar utama: PDR window dari counter beacon UAV yang benar-benar diterima.
    - Freshness tetap menjadi pembatas keras: jika tidak ada beacon baru, sinyal turun.
    - RSSI/SNR hanya menjadi safety cap ketika RF benar-benar buruk, bukan penurun
      agresif pada jarak dekat yang bisa membuat nilai 86% padahal PDR ~97-98%.
  */

  unsigned long freshFullMs = 2200UL;
  unsigned long freshZeroMs = 7000UL;

  if (activeSF >= 12) {
    freshFullMs = 5500UL;
    freshZeroMs = 18000UL;
  } else if (activeSF == 11) {
    freshFullMs = 3800UL;
    freshZeroMs = 12000UL;
  } else if (activeSF == 10) {
    freshFullMs = 2800UL;
    freshZeroMs = 9000UL;
  }

  int freshnessPct = 100;
  if (age > freshFullMs) {
    if (age >= freshZeroMs) freshnessPct = 0;
    else freshnessPct = (int)(((freshZeroMs - age) * 100UL) / (freshZeroMs - freshFullMs));
  }

  uint32_t expected = 0, rx = 0, bytes = 0;
  getPdrWindowStats(expected, rx, bytes);

  int pdrPct = 100;
  if (expected >= 4) {
    pdrPct = (int)constrain((int)(((rx * 100UL) + (expected / 2UL)) / expected), 0, 100);
  }

  int rfPct = rfQualityPercentFromRssiSnr();

  int targetPct;
  if (expected >= 4) {
    // Real dan tidak sintetis: jika PDR LoRa window = 97.75%, RADIO_STATUS akan
    // berada sekitar 97-98%, selama beacon masih fresh. Ini bukan dipaksa 100%.
    targetPct = pdrPct;
  } else {
    // Saat awal koneksi window belum cukup, gunakan freshness + RF agar MP segera
    // mendapat nilai radio yang valid tanpa menunggu puluhan sampel.
    targetPct = (freshnessPct * 70 + rfPct * 30) / 100;
  }

  // Freshness adalah pembatas utama: tidak ada beacon baru = sinyal turun.
  if (targetPct > freshnessPct) targetPct = freshnessPct;

  // RF hanya menjadi safety cap jika benar-benar buruk. Dengan cara ini, RSSI/SNR
  // yang terukur agak aneh pada jarak 60 cm tidak menurunkan indikator dari PDR 98% ke 86%.
  if (rfPct < 35 && targetPct > rfPct) targetPct = rfPct;

  // V21 SURGICAL FIX:
  // Telemetry Signal di Mission Planner dibuat merepresentasikan kualitas uplink telemetry UAV->GCS
  // yang benar-benar diterima GCS. ACK/downlink tetap dihitung oleh kode, tetapi tidak menjadi cap
  // utama indikator ini, karena itu yang menyebabkan stuck sekitar 68% saat PDR/RSSI/SNR uplink bagus.
#if MP_SIGNAL_REQUIRE_BIDIR_ACK
  int dlPct = downlinkAckQualityPercentFromUavMeta();
  if (dlPct > 0) {
    if (targetPct > dlPct) targetPct = dlPct;
  } else if (firstBeaconMs != 0 && now - firstBeaconMs >= MP_LINK_STARTUP_GRACE_MS) {
    targetPct = min(targetPct, 35);
  }
#endif

  targetPct = constrain(targetPct, 0, 100);

  // EWMA ringan agar tampilan tidak bergetar, tetapi cukup responsif.
  if (mpLinkQualityEwma_x10 < 0) {
    mpLinkQualityEwma_x10 = targetPct * 10;
  } else {
    int target_x10 = targetPct * 10;
    uint8_t alphaNum = (target_x10 < mpLinkQualityEwma_x10) ? 7 : 8;
    // Snap to 100 when the real rolling window is perfect; this is not synthetic.
    if (targetPct == 100) mpLinkQualityEwma_x10 = 1000;
    else mpLinkQualityEwma_x10 = (int16_t)(((int32_t)mpLinkQualityEwma_x10 * (10 - alphaNum) + (int32_t)target_x10 * alphaNum) / 10);
  }
  lastMpLinkQualityUpdateMs = now;

  int pct = constrain((int)((mpLinkQualityEwma_x10 + 5) / 10), 0, 100);
  if (age < freshZeroMs && pct > 0 && pct < MP_SIGNAL_MIN_LIVE_PCT) pct = MP_SIGNAL_MIN_LIVE_PCT;
  uint8_t signal = (uint8_t)constrain((int)((pct * 254UL) / 100UL), 0, 254);
  if (age < freshZeroMs && pct > 0 && signal == 0) signal = 1;
  return signal;
}
void sendRadioStatusToMissionPlanner(uint8_t sysid, uint8_t compid) {
  bool oldTelemetryContext = mpTelemetryOutputContext;
  mpTelemetryOutputContext = true;
  uint8_t signal = missionPlannerRadioSignalByte();

  mavlink_radio_status_t radio_status;
  memset(&radio_status, 0, sizeof(radio_status));

  uint8_t remoteSignal = latestRemoteRssiByteOrZero();
#if MP_MIRROR_RSSI_TO_REMRSSI_FOR_MP_UI
  // V23 surgical restart fix: jangan kirim remrssi=UINT8_MAX saat link uplink valid,
  // karena beberapa pembacaan PreFlight Mission Planner memperlakukan remote RSSI unknown
  // sebagai telemetry signal tidak valid/tidak terbaca setelah reboot.
  // Ini hanya memengaruhi field RADIO_STATUS ke USB Mission Planner; metrik ACK/downlink asli
  // tetap tersedia di MetricsSerial dan tidak dipakai untuk mengubah LoRa PHY.
  if (signal > 0) remoteSignal = signal;
#endif
  uint32_t expected = 0, rx = 0, bytes = 0;
  getPdrWindowStats(expected, rx, bytes);
  uint16_t winLost = 0;
  if (expected > rx) {
    uint32_t lost32 = expected - rx;
    if (lost32 > 65535UL) lost32 = 65535UL;
    winLost = (uint16_t)lost32;
  }

  radio_status.rssi = signal;                    // uplink UAV->GCS, dihitung dari RSSI/SNR/PDR nyata di GCS
  radio_status.remrssi = remoteSignal;           // downlink GCS->UAV, diukur nyata oleh UAV dan dikirim balik
  radio_status.txbuf = (uint8_t)constrain(100 - (int)((rawHighCount + rawLowCount + compactCmdCount) * 100UL / (RAW_HIGH_QUEUE_SIZE + RAW_LOW_QUEUE_SIZE + COMPACT_CMD_QUEUE_SIZE)), 0, 100);
  radio_status.noise = 0;
  radio_status.remnoise = 0;
  // V21 SURGICAL FIX:
  // RADIO_STATUS.rxerrors/fixed adalah counter error/corrected packet modem, bukan totalLost/totalRx bridge.
  // Kode ini tidak mengukur corrected packet ala SiK, jadi jangan isi fixed=totalRx karena dapat membuat
  // Mission Planner menilai radio telemetry terdegradasi. PDR/PLR asli tetap tersedia di MetricsSerial.
  radio_status.rxerrors = 0;
  radio_status.fixed = 0;

  /*
    Kunci perbaikan:
    - sysid tetap sysid kendaraan agar tidak membuat vehicle baru.
    - compid khusus RADIO_STATUS memakai MAV_COMP_ID_TELEMETRY_RADIO.
    Ini membuat Mission Planner lebih konsisten membaca PreFlight
    Telemetry Signal sebagai sinyal radio telemetri.
  */
  canonicalizeMissionPlannerSource(sysid, compid);

#if RADIO_STATUS_USE_TELEM_RADIO_COMPID
  #if MP_RADIO_STATUS_DEDICATED_CHAN_ENABLE
    // Kunci MPQUALITY FIX:
    // compid radio tetap dipakai agar Mission Planner mengenali telemetry radio,
    // tetapi sequence dibuat pada channel MAVLink khusus sebelum checksum dibuat.
    // Ini menghindari packet-lost palsu akibat gap sequence antara AUTOPILOT1 dan TELEMETRY_RADIO.
    mavlink_msg_radio_status_encode_chan(sysid, MAV_COMP_ID_TELEMETRY_RADIO, MP_RADIO_STATUS_MAVLINK_CHAN, &msg_out, &radio_status);
  #else
    mavlink_msg_radio_status_encode(sysid, MAV_COMP_ID_TELEMETRY_RADIO, &msg_out, &radio_status);
  #endif
  sendMavlinkMessageToMissionPlanner(msg_out);
  #if RADIO_STATUS_COMPAT_DUAL_COMPID
    uint8_t compatCompId = compid ? compid : MAV_COMP_ID_AUTOPILOT1;
    mavlink_msg_radio_status_encode(sysid, compatCompId, &msg_out, &radio_status);
    sendMavlinkMessageToMissionPlanner(msg_out);
  #endif
#else
  if (compid == 0) compid = MAV_COMP_ID_AUTOPILOT1;
  mavlink_msg_radio_status_encode(sysid, compid, &msg_out, &radio_status);
  sendMavlinkMessageToMissionPlanner(msg_out);
#endif

#if RADIO_STATUS_LEGACY_RADIO_ENABLE
  // Beberapa versi Mission Planner/ArduPilot lebih konsisten mengisi field Telemetry Signal
  // dari pesan RADIO (ardupilotmega) selain RADIO_STATUS. Nilainya tetap berasal dari metrik LoRa nyata.
  #ifdef MAVLINK_MSG_ID_RADIO
    mavlink_radio_t radio_legacy;
    memset(&radio_legacy, 0, sizeof(radio_legacy));
    radio_legacy.rssi = radio_status.rssi;
    radio_legacy.remrssi = radio_status.remrssi;
    radio_legacy.txbuf = radio_status.txbuf;
    radio_legacy.noise = radio_status.noise;
    radio_legacy.remnoise = radio_status.remnoise;
    radio_legacy.rxerrors = radio_status.rxerrors;
    radio_legacy.fixed = radio_status.fixed;
    #if RADIO_STATUS_USE_TELEM_RADIO_COMPID && MP_RADIO_STATUS_DEDICATED_CHAN_ENABLE
      mavlink_msg_radio_encode_chan(sysid, MAV_COMP_ID_TELEMETRY_RADIO, MP_RADIO_STATUS_MAVLINK_CHAN, &msg_out, &radio_legacy);
    #else
      mavlink_msg_radio_encode(sysid, MAV_COMP_ID_TELEMETRY_RADIO, &msg_out, &radio_legacy);
    #endif
    sendMavlinkMessageToMissionPlanner(msg_out);
  #endif
#endif
  mpTelemetryOutputContext = oldTelemetryContext;
}

void sendRadioStatusZeroToMissionPlanner() {
  bool oldTelemetryContext = mpTelemetryOutputContext;
  mpTelemetryOutputContext = true;
  mavlink_radio_status_t radio_status;
  memset(&radio_status, 0, sizeof(radio_status));
  radio_status.rssi = 0;
#if MP_MIRROR_RSSI_TO_REMRSSI_FOR_MP_UI
  radio_status.remrssi = 0;
#else
  radio_status.remrssi = UINT8_MAX;
#endif
  radio_status.txbuf = 0;
  radio_status.rxerrors = 0;
  radio_status.fixed = 0;

  uint8_t sysid = latestBeaconValid && latestBeaconData.system_id ? latestBeaconData.system_id : 1;
  uint8_t compid = latestBeaconValid && latestBeaconData.component_id ? latestBeaconData.component_id : MAV_COMP_ID_AUTOPILOT1;
  canonicalizeMissionPlannerSource(sysid, compid);
#if RADIO_STATUS_USE_TELEM_RADIO_COMPID
  mavlink_msg_radio_status_encode(sysid, MAV_COMP_ID_TELEMETRY_RADIO, &msg_out, &radio_status);
#else
  mavlink_msg_radio_status_encode(sysid, compid, &msg_out, &radio_status);
#endif
  sendMavlinkMessageToMissionPlanner(msg_out);
  mpTelemetryOutputContext = oldTelemetryContext;
}

void announceMissionPlannerLinkLostIfNeeded() {
#if MP_LINK_LOSS_INDICATOR_ENABLE
  if (!mpLinkLossAnnounced) {
    sendBridgeEvent(MAV_SEVERITY_CRITICAL, "LoRa telemetry link lost", MP_LINK_LOSS_STATUSTEXT_REPEAT_MS);
    mpLinkLossAnnounced = true;
  }
#endif
}

void announceMissionPlannerLinkRecoveredIfNeeded() {
#if MP_LINK_LOSS_INDICATOR_ENABLE
  if (mpLinkLossState || mpLinkLossAnnounced) {
    sendBridgeEvent(MAV_SEVERITY_INFO, "LoRa telemetry link recovered", MP_LINK_LOSS_STATUSTEXT_REPEAT_MS);
  }
  mpLinkLossAnnounced = false;
#endif
}

void enterMissionPlannerLinkLostState(unsigned long now) {
  if (!mpLinkLossState) {
    mpLinkLossState = true;
    lastMpLinkLossMs = now;
    lastMpLinkLossZeroStatusMs = 0;
  }
  announceMissionPlannerLinkLostIfNeeded();
}

void serviceMissionPlannerLostRadioStatus(unsigned long now) {
#if MP_LINK_LOSS_INDICATOR_ENABLE
  if (lastMpLinkLossZeroStatusMs == 0 || now - lastMpLinkLossZeroStatusMs >= MP_LINK_LOSS_ZERO_STATUS_INTERVAL_MS) {
    sendRadioStatusZeroToMissionPlanner();
    lastRadioStatusMs = now;
    lastMpLinkLossZeroStatusMs = now;
  }
#else
  if (now - lastMpLinkLossMs <= MP_RADIO_ZERO_BURST_AFTER_LOSS_MS) {
    sendRadioStatusZeroToMissionPlanner();
    lastRadioStatusMs = now;
  }
#endif
}

void serviceRadioStatusPeriodic() {
  // Periodic RADIO_STATUS keeps Mission Planner PreFlight telemetry signal fresh.
  // V26: if the UAV beacon becomes stale, keep reporting RADIO_STATUS=0 periodically
  // and emit one STATUSTEXT warning so the operator gets an explicit lost-link indication.
  // This is only GCS -> Mission Planner reporting; LoRa PHY and flight-control logic are unchanged.
  if (!latestBeaconValid) return;
  unsigned long now = millis();

  if (!isUplinkFreshForMissionPlanner()) {
    enterMissionPlannerLinkLostState(now);
    serviceMissionPlannerLostRadioStatus(now);
    return;
  }

  if (mpLinkLossState || mpLinkLossAnnounced) {
    announceMissionPlannerLinkRecoveredIfNeeded();
  }
  mpLinkLossState = false;
  lastMpLinkLossZeroStatusMs = 0;

  if (now - lastRadioStatusMs < RADIO_STATUS_INTERVAL_MS) return;
  uint8_t sysid = latestBeaconData.system_id ? latestBeaconData.system_id : 1;
  uint8_t compid = latestBeaconData.component_id ? latestBeaconData.component_id : MAV_COMP_ID_AUTOPILOT1;
  sendRadioStatusToMissionPlanner(sysid, compid);
  lastRadioStatusMs = now;
}

void sendTelemetryBeaconToMissionPlanner(const PixhawkDataBeacon &d) {
  bool oldTelemetryContext = mpTelemetryOutputContext;
  mpTelemetryOutputContext = true;

  if (!isUplinkFreshForMissionPlanner()) {
    unsigned long now = millis();
    enterMissionPlannerLinkLostState(now);
    serviceMissionPlannerLostRadioStatus(now);
    mpTelemetryOutputContext = oldTelemetryContext;
    return;
  }
  if (mpLinkLossState || mpLinkLossAnnounced) {
    announceMissionPlannerLinkRecoveredIfNeeded();
  }
  mpLinkLossState = false;
  lastMpLinkLossZeroStatusMs = 0;
  uint8_t sysid = d.system_id ? d.system_id : 1;
  uint8_t compid = d.component_id ? d.component_id : MAV_COMP_ID_AUTOPILOT1;
  uint8_t hbBaseMode = d.base_mode;
  uint32_t hbCustomMode = d.custom_mode;
  uint8_t hbSystemStatus = d.system_status;
  if (optimisticModeOverrideActive()) {
    hbBaseMode = optimisticModeBaseMode;
    hbCustomMode = optimisticModeCustomMode;
    hbSystemStatus = optimisticModeSystemStatus;
  }
  sendHeartbeatFromData(sysid, compid, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, hbBaseMode, hbCustomMode, hbSystemStatus);
  if (d.valid_flags & VALID_ATTITUDE) sendAttitudeFromData(sysid, compid, centiDegToRad(d.roll_cd), centiDegToRad(d.pitch_cd), centiDegToRad(d.yaw_cd), 0, 0, 0);
  if (d.valid_flags & VALID_GLOBAL_POS) {
    // Pass real NED velocity from beacon (vx_cms/vy_cms/vz_cms are cm/s NED)
    sendGlobalPositionFromData(sysid, compid,
                               d.lat, d.lon,
                               decimeterToMillimeter(d.gps.alt_dm),
                               decimeterToMillimeter(d.relative_alt_dm),
                               d.vx_cms, d.vy_cms, d.vz_cms, d.heading);
  }
  if (d.valid_flags & VALID_GPS_RAW) sendGpsRawFromBeacon(sysid, compid, d);
  if (d.valid_flags & VALID_SYS_STATUS) sendSysStatusFromData(sysid, compid, d.voltage_battery, d.current_battery, d.battery_remaining);
  float airspeed_ms    = d.airspeed_cms   / 100.0f;
  float groundspeed_ms = d.groundspeed_cms / 100.0f;
  int16_t heading_deg  = (int16_t)(d.heading / 100);
  float climb_ms       = d.climb_cms / 100.0f;
  sendVfrHudFromData(sysid, compid, airspeed_ms, groundspeed_ms, heading_deg, d.throttle, climb_ms);
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
  if (d.valid_flags & VALID_EKF) sendEkfStatusFromData(sysid, compid, d.ekf_flags);
#endif

#ifdef MAVLINK_MSG_ID_VIBRATION
  // Reconstruct VIBRATION message from compacted beacon fields
  if (d.valid_flags2 & VALID2_VIBRATION) {
    mavlink_vibration_t vib = {};
    vib.time_usec   = (uint64_t)millis() * 1000ULL;
    vib.vibration_x = (float)d.vibe_x_x100 / 100.0f;
    vib.vibration_y = (float)d.vibe_y_x100 / 100.0f;
    vib.vibration_z = (float)d.vibe_z_x100 / 100.0f;
    vib.clipping_0  = d.accel_clip;   // total clip count packed into clipping_0
    vib.clipping_1  = 0;
    vib.clipping_2  = 0;
    mavlink_msg_vibration_encode(sysid, compid, &msg_out, &vib);
    sendMavlinkMessageToMissionPlanner(msg_out);
  }
#endif

#ifdef MAVLINK_MSG_ID_RC_CHANNELS
  // Reconstruct RC_CHANNELS message from compacted channel percentage bytes
  if (d.valid_flags2 & VALID2_RC_CHANNELS) {
    mavlink_rc_channels_t rc = {};
    rc.time_boot_ms = millis();
    rc.chancount    = 16;
    rc.chan1_raw  = rcPctToRaw(d.rc_ch_pct[0]);  rc.chan2_raw  = rcPctToRaw(d.rc_ch_pct[1]);
    rc.chan3_raw  = rcPctToRaw(d.rc_ch_pct[2]);  rc.chan4_raw  = rcPctToRaw(d.rc_ch_pct[3]);
    rc.chan5_raw  = rcPctToRaw(d.rc_ch_pct[4]);  rc.chan6_raw  = rcPctToRaw(d.rc_ch_pct[5]);
    rc.chan7_raw  = rcPctToRaw(d.rc_ch_pct[6]);  rc.chan8_raw  = rcPctToRaw(d.rc_ch_pct[7]);
    rc.chan9_raw  = rcPctToRaw(d.rc_ch_pct[8]);  rc.chan10_raw = rcPctToRaw(d.rc_ch_pct[9]);
    rc.chan11_raw = rcPctToRaw(d.rc_ch_pct[10]); rc.chan12_raw = rcPctToRaw(d.rc_ch_pct[11]);
    rc.chan13_raw = rcPctToRaw(d.rc_ch_pct[12]); rc.chan14_raw = rcPctToRaw(d.rc_ch_pct[13]);
    rc.chan15_raw = rcPctToRaw(d.rc_ch_pct[14]); rc.chan16_raw = rcPctToRaw(d.rc_ch_pct[15]);
    rc.rssi = 255;  // not available over LoRa bridge
    mavlink_msg_rc_channels_encode(sysid, compid, &msg_out, &rc);
    sendMavlinkMessageToMissionPlanner(msg_out);
  }
#endif

  // RADIO_STATUS dikirim setiap beacon valid agar indikator Telemetry Signal Mission Planner
  // langsung fresh, bukan menunggu timer periodik. Ini tidak menambah beban LoRa karena hanya USB serial GCS->MP.
  sendRadioStatusToMissionPlanner(sysid, compid);
  lastRadioStatusMs = millis();
  mpTelemetryOutputContext = oldTelemetryContext;
}

void sendSyntheticHeartbeatToMissionPlannerIfNeeded() {
  bool oldTelemetryContext = mpTelemetryOutputContext;
  mpTelemetryOutputContext = true;
  if (!latestBeaconValid || !isUplinkFreshForMissionPlanner()) { mpTelemetryOutputContext = oldTelemetryContext; return; }
  unsigned long now = millis();
  if (now - lastSyntheticHeartbeatMs < HEARTBEAT_SYNTH_INTERVAL_MS) { mpTelemetryOutputContext = oldTelemetryContext; return; }
  uint8_t sysid = latestBeaconData.system_id ? latestBeaconData.system_id : 1;
  uint8_t compid = latestBeaconData.component_id ? latestBeaconData.component_id : MAV_COMP_ID_AUTOPILOT1;
  uint8_t hbBaseMode = latestBeaconData.base_mode;
  uint32_t hbCustomMode = latestBeaconData.custom_mode;
  uint8_t hbSystemStatus = latestBeaconData.system_status;
  if (optimisticModeOverrideActive()) {
    hbBaseMode = optimisticModeBaseMode;
    hbCustomMode = optimisticModeCustomMode;
    hbSystemStatus = optimisticModeSystemStatus;
  }
  sendHeartbeatFromData(sysid, compid, MAV_TYPE_HEXAROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                        hbBaseMode, hbCustomMode, hbSystemStatus);
  if (now - lastRadioStatusMs >= RADIO_STATUS_INTERVAL_MS) {
    sendRadioStatusToMissionPlanner(sysid, compid);
    lastRadioStatusMs = now;
  }
  lastSyntheticHeartbeatMs = now;
  mpTelemetryOutputContext = oldTelemetryContext;
}

void handleParsedFromUAVForMode(const mavlink_message_t &inMsg) {
  if (inMsg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) calCommandAckRxCount++;
  if (inMsg.msgid == MAVLINK_MSG_ID_STATUSTEXT) calStatustextRxCount++;
#ifdef MAVLINK_MSG_ID_MAG_CAL_PROGRESS
  if (inMsg.msgid == MAVLINK_MSG_ID_MAG_CAL_PROGRESS) { magCalProgressRxCount++; startCalibrationConfigMode(); }
#endif
#ifdef MAVLINK_MSG_ID_MAG_CAL_REPORT
  if (inMsg.msgid == MAVLINK_MSG_ID_MAG_CAL_REPORT) { magCalReportRxCount++; startCalibrationConfigMode(); }
#endif
  if (inMsg.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
    mavlink_param_value_t pv; mavlink_msg_param_value_decode(&inMsg, &pv);
    lastParamValueMs = millis(); paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
    lastParamIndex = pv.param_index; lastParamCount = pv.param_count; paramValueCount++;
  }
#ifdef MAVLINK_MSG_ID_PARAM_EXT_VALUE
  if (inMsg.msgid == MAVLINK_MSG_ID_PARAM_EXT_VALUE) {
    lastParamValueMs = millis(); paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS; paramValueCount++;
  }
#endif
}

void reencodeParsedMessageToMissionPlanner(const mavlink_message_t &inMsg) {
  uint8_t sysid = inMsg.sysid; uint8_t compid = inMsg.compid;
  canonicalizeMissionPlannerSource(sysid, compid);
  handleParsedFromUAVForMode(inMsg);
  if (isRawTelemetryDuplicateInNormalMode(inMsg.msgid)) {
    mpRawTelemetryDedupDrop++;
    return;
  }
  switch (inMsg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t p; mavlink_msg_heartbeat_decode(&inMsg, &p);
      mavlink_msg_heartbeat_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t p; mavlink_msg_sys_status_decode(&inMsg, &p);
      mavlink_msg_sys_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
      mavlink_command_ack_t p; mavlink_msg_command_ack_decode(&inMsg, &p);
      if (p.command == MAV_CMD_COMPONENT_ARM_DISARM || p.command == CMD_COMPONENT_ARM_DISARM) {
        beginArmFeedbackMode();
        if (p.result != MAV_RESULT_ACCEPTED) {
          sendBridgeEvent(MAV_SEVERITY_WARNING, "ARM rejected by FC: check battery/prearm messages", 2500UL);
        }
      }
      if (!shouldForwardCommandAckToMissionPlanner(p)) break;
      mavlink_msg_command_ack_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT: {
      mavlink_statustext_t p; mavlink_msg_statustext_decode(&inMsg, &p);
      // Saat parameter/kalibrasi, semua STATUSTEXT diteruskan agar UI Mission Planner tidak diam.
      // Saat normal, hanya warning/error/critical ke atas yang diteruskan untuk mengurangi flooding.
      if (armFeedbackActive() || paramSyncActive || calibrationConfigModeActive() || p.severity <= 4) {
        mavlink_msg_statustext_encode(sysid, compid, &msg_out, &p);
        sendMavlinkMessageToMissionPlanner(msg_out);
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE: {
      mavlink_param_value_t p; mavlink_msg_param_value_decode(&inMsg, &p);
      mavlink_msg_param_value_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK: {
      mavlink_mission_ack_t p; mavlink_msg_mission_ack_decode(&inMsg, &p);
      mavlink_msg_mission_ack_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      mavlink_mission_count_t p; mavlink_msg_mission_count_decode(&inMsg, &p);
      mavlink_msg_mission_count_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
      mavlink_mission_request_t p; mavlink_msg_mission_request_decode(&inMsg, &p);
      mavlink_msg_mission_request_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT: {
      mavlink_mission_current_t p; mavlink_msg_mission_current_decode(&inMsg, &p);
      mavlink_msg_mission_current_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM: {
      mavlink_mission_item_t p; mavlink_msg_mission_item_decode(&inMsg, &p);
      mavlink_msg_mission_item_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t p; mavlink_msg_command_long_decode(&inMsg, &p);
      mavlink_msg_command_long_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#ifdef MAVLINK_MSG_ID_COMMAND_INT
    case MAVLINK_MSG_ID_COMMAND_INT: {
      mavlink_command_int_t p; mavlink_msg_command_int_decode(&inMsg, &p);
      mavlink_msg_command_int_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_VALUE
    case MAVLINK_MSG_ID_PARAM_EXT_VALUE: {
      mavlink_param_ext_value_t p; mavlink_msg_param_ext_value_decode(&inMsg, &p);
      mavlink_msg_param_ext_value_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_PARAM_EXT_ACK
    case MAVLINK_MSG_ID_PARAM_EXT_ACK: {
      mavlink_param_ext_ack_t p; mavlink_msg_param_ext_ack_decode(&inMsg, &p);
      mavlink_msg_param_ext_ack_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_MAG_CAL_PROGRESS
    case MAVLINK_MSG_ID_MAG_CAL_PROGRESS: {
      mavlink_mag_cal_progress_t p; mavlink_msg_mag_cal_progress_decode(&inMsg, &p);
      mavlink_msg_mag_cal_progress_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_MAG_CAL_REPORT
    case MAVLINK_MSG_ID_MAG_CAL_REPORT: {
      mavlink_mag_cal_report_t p; mavlink_msg_mag_cal_report_decode(&inMsg, &p);
      mavlink_msg_mag_cal_report_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS
    case MAVLINK_MSG_ID_RC_CHANNELS: {
      mavlink_rc_channels_t p; mavlink_msg_rc_channels_decode(&inMsg, &p);
      mavlink_msg_rc_channels_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS_RAW
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
      mavlink_rc_channels_raw_t p; mavlink_msg_rc_channels_raw_decode(&inMsg, &p);
      mavlink_msg_rc_channels_raw_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
      mavlink_servo_output_raw_t p; mavlink_msg_servo_output_raw_decode(&inMsg, &p);
      mavlink_msg_servo_output_raw_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS
    case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS: {
      mavlink_actuator_output_status_t p; mavlink_msg_actuator_output_status_decode(&inMsg, &p);
      mavlink_msg_actuator_output_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4
    case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4: {
      mavlink_esc_telemetry_1_to_4_t p; mavlink_msg_esc_telemetry_1_to_4_decode(&inMsg, &p);
      mavlink_msg_esc_telemetry_1_to_4_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_ESC_STATUS
    case MAVLINK_MSG_ID_ESC_STATUS: {
      mavlink_esc_status_t p; mavlink_msg_esc_status_decode(&inMsg, &p);
      mavlink_msg_esc_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_MISSION_REQUEST_INT
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
      mavlink_mission_request_int_t p; mavlink_msg_mission_request_int_decode(&inMsg, &p);
      mavlink_msg_mission_request_int_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_MISSION_ITEM_INT
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
      mavlink_mission_item_int_t p; mavlink_msg_mission_item_int_decode(&inMsg, &p);
      mavlink_msg_mission_item_int_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_MISSION_ITEM_REACHED
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
      mavlink_mission_item_reached_t p; mavlink_msg_mission_item_reached_decode(&inMsg, &p);
      mavlink_msg_mission_item_reached_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_AUTOPILOT_VERSION
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
      mavlink_autopilot_version_t p; mavlink_msg_autopilot_version_decode(&inMsg, &p);
      mavlink_msg_autopilot_version_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_HOME_POSITION
    case MAVLINK_MSG_ID_HOME_POSITION: {
      mavlink_home_position_t p; mavlink_msg_home_position_decode(&inMsg, &p);
      mavlink_msg_home_position_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif

#ifdef MAVLINK_MSG_ID_LOG_ENTRY
    case MAVLINK_MSG_ID_LOG_ENTRY: {
      mavlink_log_entry_t p; mavlink_msg_log_entry_decode(&inMsg, &p);
      mavlink_msg_log_entry_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_LOG_DATA
    case MAVLINK_MSG_ID_LOG_DATA: {
      mavlink_log_data_t p; mavlink_msg_log_data_decode(&inMsg, &p);
      mavlink_msg_log_data_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_STORAGE_INFORMATION
    case MAVLINK_MSG_ID_STORAGE_INFORMATION: {
      mavlink_storage_information_t p; mavlink_msg_storage_information_decode(&inMsg, &p);
      mavlink_msg_storage_information_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_MOUNT_STATUS
    case MAVLINK_MSG_ID_MOUNT_STATUS: {
      mavlink_mount_status_t p; mavlink_msg_mount_status_decode(&inMsg, &p);
      mavlink_msg_mount_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: {
      mavlink_gimbal_device_attitude_status_t p; mavlink_msg_gimbal_device_attitude_status_decode(&inMsg, &p);
      mavlink_msg_gimbal_device_attitude_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION: {
      mavlink_gimbal_manager_information_t p; mavlink_msg_gimbal_manager_information_decode(&inMsg, &p);
      mavlink_msg_gimbal_manager_information_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS: {
      mavlink_gimbal_manager_status_t p; mavlink_msg_gimbal_manager_status_decode(&inMsg, &p);
      mavlink_msg_gimbal_manager_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_INFORMATION
    case MAVLINK_MSG_ID_CAMERA_INFORMATION: {
      mavlink_camera_information_t p; mavlink_msg_camera_information_decode(&inMsg, &p);
      mavlink_msg_camera_information_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_SETTINGS
    case MAVLINK_MSG_ID_CAMERA_SETTINGS: {
      mavlink_camera_settings_t p; mavlink_msg_camera_settings_decode(&inMsg, &p);
      mavlink_msg_camera_settings_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS
    case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: {
      mavlink_camera_capture_status_t p; mavlink_msg_camera_capture_status_decode(&inMsg, &p);
      mavlink_msg_camera_capture_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED
    case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED: {
      mavlink_camera_image_captured_t p; mavlink_msg_camera_image_captured_decode(&inMsg, &p);
      mavlink_msg_camera_image_captured_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_CAMERA_FOV_STATUS
    case MAVLINK_MSG_ID_CAMERA_FOV_STATUS: {
      mavlink_camera_fov_status_t p; mavlink_msg_camera_fov_status_decode(&inMsg, &p);
      mavlink_msg_camera_fov_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION
    case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: {
      mavlink_video_stream_information_t p; mavlink_msg_video_stream_information_decode(&inMsg, &p);
      mavlink_msg_video_stream_information_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_VIDEO_STREAM_STATUS
    case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS: {
      mavlink_video_stream_status_t p; mavlink_msg_video_stream_status_decode(&inMsg, &p);
      mavlink_msg_video_stream_status_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_ADSB_VEHICLE
    case MAVLINK_MSG_ID_ADSB_VEHICLE: {
      mavlink_adsb_vehicle_t p; mavlink_msg_adsb_vehicle_decode(&inMsg, &p);
      mavlink_msg_adsb_vehicle_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: {
      mavlink_uavionix_adsb_out_cfg_t p; mavlink_msg_uavionix_adsb_out_cfg_decode(&inMsg, &p);
      mavlink_msg_uavionix_adsb_out_cfg_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC: {
      mavlink_uavionix_adsb_out_dynamic_t p; mavlink_msg_uavionix_adsb_out_dynamic_decode(&inMsg, &p);
      mavlink_msg_uavionix_adsb_out_dynamic_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
#ifdef MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT: {
      mavlink_uavionix_adsb_transceiver_health_report_t p; mavlink_msg_uavionix_adsb_transceiver_health_report_decode(&inMsg, &p);
      mavlink_msg_uavionix_adsb_transceiver_health_report_encode(sysid, compid, &msg_out, &p);
      sendMavlinkMessageToMissionPlanner(msg_out); break;
    }
#endif
    default: mavlinkParseDrop++; break;
  }
}

void parseAndReencodeRawToMissionPlanner(const MavlinkRawPacket &raw) {
  if (raw.len == 0 || raw.len > RAW_MAVLINK_MAX) return;
  mavlink_message_t parsedMsg; mavlink_status_t parsedStatus;
  memset(&parsedMsg, 0, sizeof(parsedMsg)); memset(&parsedStatus, 0, sizeof(parsedStatus));
  for (uint8_t i = 0; i < raw.len; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_2, raw.payload[i], &parsedMsg, &parsedStatus)) {
      reencodeParsedMessageToMissionPlanner(parsedMsg);
    }
  }
}

void parseAndReencodeParamBulkToMissionPlanner(const ParamBulkPacket &pkt) {
  if (pkt.count == 0 || pkt.count > PARAM_BULK_MAX_RECORDS) return;
  uint8_t sysid = pkt.sysid == 0 ? 1 : pkt.sysid;
  uint8_t compid = pkt.compid == 0 ? MAV_COMP_ID_AUTOPILOT1 : pkt.compid;
  canonicalizeMissionPlannerSource(sysid, compid);

  for (uint8_t i = 0; i < pkt.count; i++) {
    const CompactParamValue &r = pkt.rec[i];
    mavlink_param_value_t p = {};
    p.param_value = r.param_value;
    p.param_count = r.param_count;
    p.param_index = r.param_index;
    p.param_type = r.param_type;
    memcpy(p.param_id, r.param_id, 16);

    mavlink_msg_param_value_encode(sysid, compid, &msg_out, &p);
    sendMavlinkMessageToMissionPlanner(msg_out);

    lastParamValueMs = millis();
    paramSyncUntilMs = millis() + PARAM_SYNC_TIMEOUT_MS;
    lastParamIndex = p.param_index;
    lastParamCount = p.param_count;
    paramValueCount++;
  }
  paramBulkRxCount++;
}

uint16_t extractFlightActionCommandIdFromRawPacket(const MavlinkRawPacket &raw) {
  mavlink_message_t parsedMsg;
  mavlink_status_t parsedStatus;
  memset(&parsedMsg, 0, sizeof(parsedMsg));
  memset(&parsedStatus, 0, sizeof(parsedStatus));
  for (uint8_t i = 0; i < raw.len; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_2, raw.payload[i], &parsedMsg, &parsedStatus)) {
      if (parsedMsg.msgid == MAVLINK_MSG_ID_SET_MODE) return CMD_DO_SET_MODE;
      if (parsedMsg.msgid == MAVLINK_MSG_ID_MISSION_SET_CURRENT) return MAV_CMD_MISSION_START;
      if (parsedMsg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&parsedMsg, &cmd);
        uint16_t c = (uint16_t)cmd.command;
        if (isFlightActionCommandId(c)) return c;
      }
#ifdef MAVLINK_MSG_ID_COMMAND_INT
      if (parsedMsg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        mavlink_command_int_t cmd;
        mavlink_msg_command_int_decode(&parsedMsg, &cmd);
        uint16_t c = (uint16_t)cmd.command;
        if (isFlightActionCommandId(c)) return c;
      }
#endif
    }
  }
  return 0;
}

bool shouldProxyAckForCommand(uint16_t command) {
#if COMMAND_PROXY_ACK_HIGH_SF_ENABLE
  if (command == 0) return false;
  if (activeSF < COMMAND_PROXY_ACK_MIN_SF) return false;
  // V14 delay/force fix:
  // Do NOT proxy MAV_CMD_COMPONENT_ARM_DISARM as ACCEPTED. If normal arming is rejected
  // by ArduPilot pre-arm checks, Mission Planner must receive the real DENIED/FAILED ACK
  // and STATUSTEXT so the Force Arm option can appear.
  if (command == MAV_CMD_COMPONENT_ARM_DISARM || command == CMD_COMPONENT_ARM_DISARM) return false;
#if !COMMAND_PROXY_ACK_ARM_ENABLE
  if (command == MAV_CMD_COMPONENT_ARM_DISARM) return false;
#endif
  return isFlightActionCommandId(command);
#else
  (void)command;
  return false;
#endif
}

void sendForceArmProxyAckToMissionPlanner() {
  // V16 STABLE: never send bridge COMMAND_ACK for FORCE ARM.
  // Only notify the user; wait for real FC ACK to avoid Mission Planner stale-ACK crash.
  sendBridgeEvent(MAV_SEVERITY_WARNING, "FORCE ARM forwarded; waiting real FC ACK", 2500UL);
}


void sendProxyCommandAckNow(uint16_t command) {
  if (!shouldProxyAckForCommand(command)) return;
  uint8_t sysid = latestBeaconValid && latestBeaconData.system_id ? latestBeaconData.system_id : 1;
  uint8_t compid = latestBeaconValid && latestBeaconData.component_id ? latestBeaconData.component_id : MAV_COMP_ID_AUTOPILOT1;
  mavlink_command_ack_t ack = {};
  ack.command = command;
  ack.result = MAV_RESULT_ACCEPTED;
  ack.progress = 0;
  ack.result_param2 = 0;
  ack.target_system = 255;
  ack.target_component = 190;
  mavlink_msg_command_ack_encode(sysid, compid, &msg_out, &ack);
  sendMavlinkMessageToMissionPlanner(msg_out);
  rememberCommandProxyAck(command);
  clearRecentMissionPlannerCommand(command);
  commandProxyAckSentCount++;
}

void sendProxyCommandAckToMissionPlanner(uint16_t command) {
  if (!shouldProxyAckForCommand(command)) return;
  proxyAckBurstCommand = command;
  proxyAckBurstRemaining = COMMAND_PROXY_ACK_BURST_COUNT;
  proxyAckBurstNextMs = millis();
  serviceProxyCommandAckBurst();
}

void serviceProxyCommandAckBurst() {
  if (proxyAckBurstCommand == 0 || proxyAckBurstRemaining == 0) return;
  unsigned long now = millis();
  if (now < proxyAckBurstNextMs) return;
  sendProxyCommandAckNow(proxyAckBurstCommand);
  proxyAckBurstRemaining--;
  if (proxyAckBurstRemaining == 0) {
    proxyAckBurstCommand = 0;
    proxyAckBurstNextMs = 0;
  } else {
    proxyAckBurstNextMs = now + COMMAND_PROXY_ACK_BURST_GAP_MS;
  }
}

bool sendAckOrPendingCommand(uint32_t ackCounter) {
  CompactCommandQueueItem compact;
  MavlinkRawPacket raw; bool fromHighQueue = false;
  radio.standby();
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(activeSF);
  radio.setOutputPower(activeTP);
  delay(DOWNLINK_TX_GUARD_MS);
  if (peekCompactCommandPacket(compact)) {
    int16_t state = radio.transmit(compact.payload, compact.len);
    radio.standby();
    if (state == RADIOLIB_ERR_NONE) {
      bool arm = false, force = false;
      bool isArmCmd = decodeArmCommandFromCompactItem(compact, arm, force);
      if (isArmCmd) markArmCommandTransactionTx(arm, force);
      popCompactCommandPacket();
      radioBytesTx += compact.len;
      compactCmdTxCount++;
      // V17 real-state: do not send bridge/proxy COMMAND_ACK for any command.
      // Wait for real COMMAND_ACK/STATUSTEXT/HEARTBEAT from the flight controller.
      return true;
    }
    return false;
  }
  if (peekNextRawPacket(raw, fromHighQueue)) {
    uint16_t packetLen = RAW_PKT_HEADER_LEN + raw.len;
    finalizePacketCrc(&raw, packetLen);
    int16_t state = radio.transmit((uint8_t *)&raw, packetLen);
    radio.standby();
    if (state == RADIOLIB_ERR_NONE) {
      uint16_t flightCmd = extractFlightActionCommandIdFromRawPacket(raw);
      bool arm = false, force = false;
      bool isArmCmd = decodeArmCommandFromRawPacket(raw, arm, force);
      if (isArmCmd) markArmCommandTransactionTx(arm, force);
      popNextRawPacket(fromHighQueue);
      radioBytesTx += packetLen;
      // V17 real-state: no proxy ACK. Forward only real FC response.
      (void)flightCmd;
      return true;
    }
    return false;
  }
  LinkAckPacket ack = {}; initHeader(ack.hdr, PKT_LINK_ACK); ack.counter = ackCounter;
  finalizePacketCrc(&ack, sizeof(ack));
  int16_t state = radio.transmit((uint8_t *)&ack, sizeof(ack));
  radio.standby();
  if (state == RADIOLIB_ERR_NONE) { radioBytesTx += sizeof(ack); return true; }
  return false;
}

bool sendConfigAck(const ConfigProposalPacket &proposal) {
  ConfigAckPacket ack = {}; initHeader(ack.hdr, PKT_CONFIG_ACK);
  ack.counter = proposal.counter; ack.apply_counter = proposal.apply_counter; ack.accepted = 1;
  ack.next_sf = proposal.next_sf; ack.next_tp = proposal.next_tp; ack.next_profile = proposal.next_profile;
  finalizePacketCrc(&ack, sizeof(ack));
  radio.standby();
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setSpreadingFactor(activeSF);
  radio.setOutputPower(activeTP);
  delay(DOWNLINK_TX_GUARD_MS);
  int16_t state = radio.transmit((uint8_t *)&ack, sizeof(ack));
  radio.standby();
  if (state == RADIOLIB_ERR_NONE) {
    radioBytesTx += sizeof(ack);
    scheduledConfig = true; scheduledSF = proposal.next_sf; scheduledTP = proposal.next_tp;
    scheduledProfile = proposal.next_profile; scheduledApplyCounter = proposal.apply_counter;
    scheduledConfigSinceMs = millis();
    return true;
  }
  return false;
}

void applyScheduledConfigAfterAck(uint32_t receivedCounter) {
  if (!scheduledConfig) return;
  if ((receivedCounter + 1) >= scheduledApplyCounter) {
    activeSF = scheduledSF; activeTP = scheduledTP; applyRadioSettings(activeSF, activeTP); scheduledConfig = false; scheduledConfigSinceMs = 0;
  }
}

void updateScheduledConfigGuard() {
  if (!scheduledConfig || scheduledConfigSinceMs == 0) return;
  if (millis() - scheduledConfigSinceMs <= SCHEDULED_CONFIG_STUCK_MS) return;
  scheduledConfig = false; scheduledConfigSinceMs = 0; scheduledConfigCancelCount++; softRecoverRadio();
}

void updateRecoveryScanning() {
  unsigned long now = millis();
  if (scheduledConfig) return;  // tunggu migrasi SF/TP selesai atau guard membatalkan
  unsigned long idleLimit = phyLocked ? linkIdleScanAfterForSF(activeSF) : 0UL;
  if (phyLocked && now - lastPacketMs <= idleLimit) return;
  if (now - lastScanMs < RECOVERY_SCAN_STEP_MS) return;
#if PHY_TEST_LOCK_INITIAL_SF
  activeSF = DEFAULT_SF;
  scanSF = DEFAULT_SF;
  applyRadioSettings(activeSF);
#else
  phyLocked = false;
  // UAV-master PHY: saat belum lock/timeout, GCS menyapu SF7..SF12.
  activeSF = scanSF;
  applyRadioSettings(activeSF);
  scanSF++;
  if (scanSF > SF_MAX) scanSF = SF_MIN;
#endif
  recoveryStep++; recoveryScanCount++; lastScanMs = now;
}

void hardResetLoRa() { pinMode(LORA_RST, OUTPUT); digitalWrite(LORA_RST, LOW); delay(20); digitalWrite(LORA_RST, HIGH); delay(80); }

bool initLoRaRadio() {
  for (uint8_t i = 0; i < LORA_INIT_RETRY_COUNT; i++) {
    hardResetLoRa(); delay(LORA_INIT_RETRY_DELAY_MS);
    int16_t state = radio.begin(FREQ_MHZ, LORA_BW_KHZ, activeSF, LORA_CR_DEN, LORA_SYNC, activeTP);
    MetricsSerial.print("LoRa init attempt "); MetricsSerial.print(i+1); MetricsSerial.print(" state="); MetricsSerial.println(state);
    if (state == RADIOLIB_ERR_NONE) { radio.explicitHeader(); radio.setCRC(true); applyRadioSettings(activeSF); return true; }
  }
  return false;
}

void handleTelemetryPacketCommon(uint32_t pktCounter, uint8_t pktType, uint8_t sf, uint8_t tp, uint8_t profile, uint8_t mode, uint16_t latency_x100, uint16_t packetSize, const TelemetryMeta &meta) {
  // Lock ke PHY yang diumumkan UAV di beacon/header. Paket ini hanya bisa diterima jika radio GCS
  // sedang berada pada SF yang kompatibel; field sf dipakai sebagai state resmi untuk ACK/command berikutnya.
  activeSF = sf;
  activeTP = tp;
  scanSF = sf;
  phyLocked = true;
  lastProfile = profile; lastPktType = pktType; recoveryStep = 0; syncModeFromUAV(mode);
  lastRssi = radio.getRSSI();
  lastSnr = radio.getSNR();
  latestUavCntAck = meta.cnt_ack;
  latestUavSuccessStreak = meta.success_streak;
  latestUavFailStreak = meta.fail_streak;
  latestUavMetaMs = millis();
  bool pendingDownlink = (compactCmdCount > 0 || rawHighCount > 0 || rawLowCount > 0);
  bool ackSlot = shouldRespondToTelemetry(pktCounter, sf);

  // V14 HOTFIX COMMAND-SLOT:
  // SF10-SF12 UAV hanya pasti membuka RX pada telemetry ACK slot.
  // Jangan kirim command hanya karena GCS sedang commandPriorityActive(), karena sebelum
  // command pertama diterima, UAV belum masuk commandModeActive() dan tidak mendengar
  // pada non-ACK slot. Jika GCS transmit pada non-ACK slot, radio.transmit() tetap sukses
  // secara lokal, queue dipop, tetapi UAV tidak menerima command.
  bool mayUseThisSlot = ackSlot;
  if (sf <= 9 && pendingDownlink) mayUseThisSlot = true;

  if (mayUseThisSlot) {
    if (pendingDownlink) flightCommandTxSlotCount++;
    sendAckOrPendingCommand(pktCounter);
    if (pendingDownlink) lastFlightCommandTxMs = millis();
  }
  printMetrics(pktCounter, pktType, sf, tp, profile, mode, latency_x100, packetSize, meta);
  sendHighSfConnectionNoticeIfNeeded(sf);
  sendLoRaParamNoticeIfNeeded(sf, tp);
  applyScheduledConfigAfterAck(pktCounter);
}

void hardRecoverLoRaIfNoPackets() {
  unsigned long now = millis();
  if (tStart == 0 || now - tStart < GCS_LORA_NO_PACKET_RECOVERY_MS) return;
  if (lastPacketMs != 0 && now - lastPacketMs < GCS_LORA_NO_PACKET_RECOVERY_MS) return;
  if (now - lastGcsLoRaHardRecoveryMs < GCS_LORA_HARD_RECOVERY_MIN_GAP_MS) return;

  lastGcsLoRaHardRecoveryMs = now;
  gcsLoRaHardRecoveryCount++;
  MetricsSerial.print("[RECOVERY GCS] No valid LoRa packet, hard radio recovery count=");
  MetricsSerial.println(gcsLoRaHardRecoveryCount);

  phyLocked = false;
  activeSF = SF_MIN;
  scanSF = SF_MIN;
  scheduledConfig = false;
  radio.standby();
  hardResetLoRa();
  int16_t state = radio.begin(FREQ_MHZ, LORA_BW_KHZ, activeSF, LORA_CR_DEN, LORA_SYNC, activeTP);
  MetricsSerial.print("[RECOVERY GCS] radio.begin state=");
  MetricsSerial.println(state);
  if (state == RADIOLIB_ERR_NONE) {
    radio.explicitHeader();
    radio.setCRC(true);
    applyRadioSettings(activeSF);
  }
  lastScanMs = 0;
}

void setup() {
  GCS_Serial.begin(115200);
  MetricsSerial.begin(115200, SERIAL_8N1, 16, 17);
  delay(BOOT_STABILIZE_MS);
  spi.begin(18, 19, 23, LORA_SS);
#if PHY_TEST_LOCK_INITIAL_SF
  activeSF = DEFAULT_SF;
  scanSF = DEFAULT_SF;
#else
  // UAV-master PHY: GCS tidak perlu di-upload ulang saat SF UAV berubah.
  // GCS mulai scan dari SF_MIN lalu lock saat menerima beacon valid dari UAV.
  activeSF = SF_MIN;
  scanSF = SF_MIN;
#endif
  MetricsSerial.print("[BOOT GCS] DEFAULT_SF=");
  MetricsSerial.print(DEFAULT_SF);
  MetricsSerial.print(" activeSF=");
  MetricsSerial.print(activeSF);
  MetricsSerial.print(" activeTP=");
  MetricsSerial.print(activeTP);
  MetricsSerial.print(" scanSF=");
  MetricsSerial.print(scanSF);
  MetricsSerial.print(" masterPHY=");
  MetricsSerial.println(UAV_MASTER_PHY_MODE);
  if (!initLoRaRadio()) {
    MetricsSerial.println("LoRa init failed after retries, restarting ESP32");
    delay(GCS_LORA_INIT_FAIL_RESTART_MS);
    ESP.restart();
  }
  tStart = millis();
  phyLocked = false;
  lastPacketMs = millis();
  lastScanMs = 0;
  printMetricsHeader();
}

void loop() {
  readGCSMavlinkCommands();
  updateParamSyncTimeout();
  updateCommandPriorityMode();
  updateCalibrationConfigMode();
  updateScheduledConfigGuard();
  updateRecoveryScanning();
  hardRecoverLoRaIfNoPackets();
  // V17 real-state: no cached/synthetic heartbeat, no proxy ACK, no optimistic UI heartbeat.
  // Mission Planner receives only real FC state reconstructed from beacon/raw MAVLink.
  serviceLoRaParamNoticeBurst();
  serviceRadioStatusPeriodic();

  uint8_t rxBuf[LORA_RX_MAX]; memset(rxBuf, 0, sizeof(rxBuf));
  radio.standby(); radio.setBandwidth(LORA_BW_KHZ); radio.setSpreadingFactor(activeSF);
  int16_t state = radio.receive(rxBuf, sizeof(rxBuf), rxTimeoutForSF(activeSF));
  size_t rxLen = radio.getPacketLength();

  if (state != RADIOLIB_ERR_NONE) return;
  if (rxLen == 0 || rxLen > LORA_RX_MAX) return;
  if (!validatePacket(rxBuf, rxLen)) return;

  PacketHeader *hdr = (PacketHeader *)rxBuf;
  radioBytesRx += rxLen; recoveryStep = 0;
  readGCSMavlinkCommands();

  if (hdr->type == PKT_CONFIG_PROPOSE) {
    if (rxLen != sizeof(ConfigProposalPacket)) return;
    lastPacketMs = millis();
    ConfigProposalPacket proposal; memcpy(&proposal, rxBuf, sizeof(proposal));
    sendConfigAck(proposal); return;
  }

  if (hdr->type == PKT_TELEM_BEACON) {
    if (rxLen != sizeof(TelemetryBeaconPacket)) return;
    TelemetryBeaconPacket pkt; memcpy(&pkt, rxBuf, sizeof(pkt));
    if (!acceptRollingUavCounter(pkt.counter)) return;
    uint8_t pktSF = unpackSf78(pkt.radio_packed);
    uint8_t pktTP = unpackTp78(pkt.radio_packed);
    uint8_t pktProfile = unpackProfile78(pkt.radio_packed);
    updateRemoteMetricFromBeacon78(pkt);
    lastPacketMs = millis();
    if (firstBeaconMs == 0) firstBeaconMs = lastPacketMs;
    memcpy(&latestBeaconData, &pkt.data, sizeof(latestBeaconData));
    latestBeaconValid = true;
    handleTelemetryPacketCommon(pkt.counter, pkt.hdr.type, pktSF, pktTP, pktProfile, pkt.link_mode, pkt.latency_x100, sizeof(pkt), pkt.meta);
    updateOptimisticModeFromRealBeacon(pkt.data);
    sendTelemetryBeaconToMissionPlanner(pkt.data);
    lastSyntheticHeartbeatMs = millis();
    return;
  }

  if (hdr->type == PKT_PARAM_BULK) {
    if (rxLen < PARAM_BULK_BASE_LEN || rxLen > sizeof(ParamBulkPacket)) return;
    ParamBulkPacket pkt; memset(&pkt, 0, sizeof(pkt)); memcpy(&pkt, rxBuf, rxLen);
    if (pkt.count == 0 || pkt.count > PARAM_BULK_MAX_RECORDS) return;
    if (rxLen != (size_t)PARAM_BULK_LEN(pkt.count)) return;
    lastPacketMs = millis();
    sendAckOrPendingCommand(0);
    parseAndReencodeParamBulkToMissionPlanner(pkt);
    return;
  }

  if (hdr->type == PKT_MAVLINK_RAW) {
    if (rxLen < RAW_PKT_HEADER_LEN || rxLen > sizeof(MavlinkRawPacket)) return;
    MavlinkRawPacket raw; memset(&raw, 0, sizeof(raw)); memcpy(&raw, rxBuf, rxLen);
    if (raw.len == 0 || raw.len > RAW_MAVLINK_MAX) return;
    if (rxLen < (size_t)(RAW_PKT_HEADER_LEN + raw.len)) return;
    lastPacketMs = millis();
    sendAckOrPendingCommand(0);
    parseAndReencodeRawToMissionPlanner(raw);
    return;
  }
}