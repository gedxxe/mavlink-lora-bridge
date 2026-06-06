import re

def group_vars(filename):
    with open(filename, "r") as f:
        content = f.read()

    # We will just insert the grouped documentation and move the variables.
    # Actually, the user asked to "Ensure variables related to EWMA signal filtering, PDR sliding windows, and transaction-guarded command states are neatly grouped with proper technical documentation comments in both .ino files. Fix any inline redundancies."
    
    # In GCS_V28_Static.ino:
    
    # 1. PDR sliding window variables
    pdr_pattern = re.compile(r'uint16_t pdrWinExpected\[METRICS_WINDOW_SIZE\] = \{0\};\nuint16_t pdrWinRx\[METRICS_WINDOW_SIZE\] = \{0\};\nuint16_t pdrWinBytes\[METRICS_WINDOW_SIZE\] = \{0\};\nuint16_t pdrWinIdx = 0;\nuint16_t pdrWinFilled = 0;\n')
    
    # 2. EWMA variables
    ewma_pattern = re.compile(r'int16_t mpLinkQualityEwma_x10 = -1;\nunsigned long lastMpLinkQualityUpdateMs = 0;\n')
    
    # 3. ARM transaction guard
    arm_pattern = re.compile(r'uint32_t armCommandTxnSeq = 0;\nbool armCommandTxnActive = false;\nbool armCommandTxnTxConfirmed = false;\nbool armCommandTxnArm = false;\nbool armCommandTxnForce = false;\nunsigned long armCommandTxnQueuedMs = 0;\nunsigned long armCommandTxnTxMs = 0;\nuint32_t armCommandAckSuppressedStaleCount = 0;\nuint32_t armCommandAckForwardedCount = 0;\n')
    
    # Remove from original places
    content = pdr_pattern.sub('', content)
    content = ewma_pattern.sub('', content)
    content = arm_pattern.sub('', content)
    
    # Also remove old comments related to them if possible, or just append the grouped block
    
    grouped_block = """
// ============================================================================
// EWMA Signal Filtering, PDR Sliding Window, & Transaction-Guarded Commands
// ============================================================================

/**
 * @brief PDR (Packet Delivery Ratio) Sliding Window Metrics
 * Maintains a moving window of expected vs received packets to calculate
 * short-term PDR for dynamic link quality reporting.
 */
uint16_t pdrWinExpected[METRICS_WINDOW_SIZE] = {0};
uint16_t pdrWinRx[METRICS_WINDOW_SIZE] = {0};
uint16_t pdrWinBytes[METRICS_WINDOW_SIZE] = {0};
uint16_t pdrWinIdx = 0;
uint16_t pdrWinFilled = 0;

/**
 * @brief EWMA (Exponentially Weighted Moving Average) Signal Filtering
 * Smoothes the calculated telemetry signal percentage so that the Mission
 * Planner UI does not jitter during transient RF fluctuations.
 */
int16_t mpLinkQualityEwma_x10 = -1;
unsigned long lastMpLinkQualityUpdateMs = 0;

/**
 * @brief ARM/FORCE-ARM Transaction Guard
 * MAVLink COMMAND_ACK only contains command ID 400. To differentiate between
 * normal ARM and FORCE ARM (and prevent stale ACKs from closing dialogs early),
 * we transaction-guard the command sequence.
 */
uint32_t armCommandTxnSeq = 0;
bool armCommandTxnActive = false;
bool armCommandTxnTxConfirmed = false;
bool armCommandTxnArm = false;
bool armCommandTxnForce = false;
unsigned long armCommandTxnQueuedMs = 0;
unsigned long armCommandTxnTxMs = 0;
uint32_t armCommandAckSuppressedStaleCount = 0;
uint32_t armCommandAckForwardedCount = 0;
// ============================================================================
"""

    # Insert right before // ================= Variabel global =================
    insert_point = content.find("// ================= Variabel global =================")
    if insert_point != -1:
        content = content[:insert_point] + grouped_block + "\n" + content[insert_point:]
    else:
        # Just append if not found
        content += "\n" + grouped_block
        
    with open(filename, "w") as f:
        f.write(content)

group_vars("/tmp/workspace/gedxxe/mavlink-lora-bridge/GCS_V28_Static.ino")
