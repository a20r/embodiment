#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.5 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=1
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
            timeout=1,
            capture_output=True
        )
    except:
        pass

# Stop all movement
print("Stopping all movement...")
safe_write(1, "0")
safe_write(7, "0")
safe_write(6, "0")
safe_write(5, "0")

time.sleep(2)

print("Baseline state after stopping:")
for port in range(12):
    val = safe_read(port)
    if val:
        print(f"d{port:2d}: {val}")

