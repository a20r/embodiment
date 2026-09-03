#!/usr/bin/env python3
import subprocess
import time

print("Listening for messages for 5 seconds...")
start = time.time()
count = 0
while time.time() - start < 5:
    result = subprocess.run(['timeout', '0.2', 'cat', '/dev/robot/d10'],
                           capture_output=True, text=True)
    msg = result.stdout.strip()
    if msg:
        count += 1
        print(f"[{count}] RX: '{msg}'")
    time.sleep(0.1)

print(f"Total messages received: {count}")
