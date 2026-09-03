#!/usr/bin/env python3
import subprocess
import time

def read_rx():
    try:
        result = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d10'],
                               capture_output=True, text=True)
        return result.stdout.strip()
    except:
        return ""

print("Listening for 20 seconds with high frequency...")
for i in range(20):
    msg = read_rx()
    if msg and len(msg) > 0:
        print(f"[{i}] RX: {msg}")
    time.sleep(1)

print("Done listening")
