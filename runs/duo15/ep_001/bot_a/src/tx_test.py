#!/usr/bin/env python3
import subprocess
import time

def tx(msg):
    # Try different message formats
    subprocess.run(f'echo "{msg}" > /dev/robot/d8', shell=True, capture_output=True, timeout=1)

def rx():
    result = subprocess.run('timeout 0.2 cat /dev/robot/d10', shell=True, capture_output=True, text=True)
    return result.stdout.strip()

print("Testing TX formats...")

# Try various formats
formats = [
    "HELLO",
    "HELLO\n",
    "HELLO ROBOT 2",
    "HELLO_ROBOT_1_HERE",
    "MEET_AT_ORIGIN",
    "SYNC_REQUEST",
    "",  # Empty message
]

for fmt in formats:
    tx(fmt)
    time.sleep(0.2)
    resp = rx()
    if resp:
        print(f"TX: '{fmt}' -> RX: '{resp}'")

print("\nListening for 5 seconds...")
for i in range(10):
    resp = rx()
    if resp:
        print(f"[{i}] RX: {resp}")
    time.sleep(0.5)

print("Done")
