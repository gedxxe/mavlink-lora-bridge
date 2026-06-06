import re

def remove_link_modes(filename):
    with open(filename, "r") as f:
        content = f.read()

    # Regex to match the Link Mode defines
    pattern = re.compile(r'// ================= Link Mode =================\n(#define LINK_MODE_.*?\n)+')
    new_content = re.sub(pattern, '', content)
    
    with open(filename, "w") as f:
        f.write(new_content)

remove_link_modes("/tmp/workspace/gedxxe/mavlink-lora-bridge/GCS_V28_Static.ino")
remove_link_modes("/tmp/workspace/gedxxe/mavlink-lora-bridge/UAV_V28_Static.ino")
