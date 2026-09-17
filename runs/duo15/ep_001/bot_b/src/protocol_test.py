#!/usr/bin/env python3
import subprocess
import time
import sys

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.1 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.3
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.3,
            capture_output=True
        )
    except:
        pass

# Try different protocol ideas
protocols = [
    ("START", "d8"),
    ("ROBOT", "d8"),
    ("R1", "d8"),
    ("R2", "d8"),
    ("GO", "d8"),
    ("MOVE", "d8"),
]

print("Testing different protocol messages...")

for msg, port_num in protocols:
    print(f"\nSending '{msg}' on {port_num}...")
    safe_write(int(port_num[1:]), msg)
    
    # Check d10 for response
    for _ in range(3):
        resp = safe_read(10)
        if resp:
            print(f"  Response on d10: {resp}")
        time.sleep(0.1)

