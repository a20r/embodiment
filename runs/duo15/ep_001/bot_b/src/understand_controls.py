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

print("Understanding d6 (turn) control...")

# First stop all movement
safe_write(1, "0")
time.sleep(1)

# Now test d6 at different values
test_values = [0, 45, 90, 135, 180, 225, 270, 315]

for value in test_values:
    safe_write(6, str(value))
    time.sleep(0.5)
    
    bearing = safe_read(4)
    d9 = safe_read(9)
    print(f"d6={value:3d} -> bearing={bearing}, d9={d9}")

