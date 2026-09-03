#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.2 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.5
        )
        if result.stdout:
            return result.stdout.strip()
    except:
        pass
    return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.5,
            capture_output=True
        )
    except:
        pass

print("Tracking position while moving...")
print()

# Stop first
safe_write(1, "0")
time.sleep(0.5)

# Get baseline
d9 = safe_read(9)
d11 = safe_read(11)
print(f"Baseline: d9={d9}, d11={d11}")

# Move forward
print("\nMoving forward for 10 seconds...")
safe_write(1, "1")

for i in range(10):
    time.sleep(1)
    d9 = safe_read(9)
    d11 = safe_read(11)
    d4 = safe_read(4)
    print(f"{i+1}: d9={d9}, d11={d11}, bearing={d4}")

safe_write(1, "0")
print("\nStopped")

