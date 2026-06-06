import re
import sys

def process(filename):
    with open(filename, "r") as f:
        content = f.read()

    # Find where the block of defines starts
    match_start = re.search(r'#ifndef UINT8_MAX|#ifndef RADIOLIB_ERR_UNKNOWN', content)
    if not match_start:
        print(f"Could not find start in {filename}")
        return

    # Find the end (the node identifier comment)
    match_end = re.search(r'// =====================================================', content[match_start.start():])
    if not match_end:
        print(f"Could not find end in {filename}")
        return
        
    start_idx = match_start.start()
    end_idx = start_idx + match_end.start()
    
    # We replace the content between start_idx and end_idx with `#include "SharedConfig.h"\n\n`
    new_content = content[:start_idx] + '#include "SharedConfig.h"\n\n' + content[end_idx:]
    
    with open(filename, "w") as f:
        f.write(new_content)
    print(f"Processed {filename}")

process("/tmp/workspace/gedxxe/mavlink-lora-bridge/GCS_V28_Static.ino")
process("/tmp/workspace/gedxxe/mavlink-lora-bridge/UAV_V28_Static.ino")
