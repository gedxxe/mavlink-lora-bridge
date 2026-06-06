import re

def remove_blocks(filename):
    with open(filename, "r") as f:
        content = f.read()

    # Protocol block
    content = re.sub(r'// ================= Protocol =================\n(#define [^\n]+\n)+', '', content)
    
    # Packet type block
    content = re.sub(r'#define PKT_LINK_ACK.*?\n(#define [^\n]+\n)+', '', content, flags=re.DOTALL|re.MULTILINE)
    
    # We might need a more targeted regex. Let's do line-by-line removal of specific defines.
    defines_to_remove = [
        "PROTOCOL_VERSION", "NETWORK_ID", "AUTH_TOKEN", "SECURITY_KEY_MIX",
        "SECURITY_ANTI_REPLAY_ENABLE", "SECURITY_MAX_COUNTER_GAP", "SECURITY_REBOOT_GRACE_MS",
        "LORA_SS", "LORA_RST", "LORA_DIO0", "LORA_DIO1", "FREQ_MHZ", "LORA_BW_KHZ",
        "LORA_CR_DEN", "LORA_SYNC", "SF_MIN", "SF_MAX", "TP_MIN", "TP_MAX",
        "PKT_LINK_ACK", "PKT_MAVLINK_RAW", "PKT_CMD_COMPACT", "PKT_PARAM_BULK",
        "PKT_TELEM_BEACON", "PKT_CONFIG_PROPOSE", "PKT_CONFIG_ACK", "PROFILE_BEACON",
        "RAW_MAVLINK_MAX", "PARAM_BULK_MAX_RECORDS", "COMPACT_CMD_MAX_LEN",
        "LORA_PREAMBLE_SYMBOLS", "LORA_PHY_CRC_ENABLED", "LORA_IMPLICIT_HEADER",
        "LORA_INIT_RETRY_COUNT", "LORA_INIT_RETRY_DELAY_MS",
        "VALID_HEARTBEAT", "VALID_ATTITUDE", "VALID_GLOBAL_POS", "VALID_VFR_HUD",
        "VALID_SYS_STATUS", "VALID_EKF", "VALID_GPS_RAW"
    ]
    
    lines = content.split('\n')
    new_lines = []
    for line in lines:
        if line.startswith('#define '):
            macro_name = line.split()[1]
            if macro_name in defines_to_remove:
                continue
        new_lines.append(line)
        
    with open(filename, "w") as f:
        f.write('\n'.join(new_lines))

remove_blocks("/tmp/workspace/gedxxe/mavlink-lora-bridge/GCS_V28_Static.ino")
remove_blocks("/tmp/workspace/gedxxe/mavlink-lora-bridge/UAV_V28_Static.ino")
