#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.3 cat /dev/robot/d{port_num}",
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

# Try writing to d0
print("Testing d0 as input...")
for val in ["0", "1", "PING", "WAKE"]:
    print(f"\nWriting d0={val}")
    safe_write(0, val)
    time.sleep(0.5)
    
    d0 = safe_read(0)
    d10 = safe_read(10)
    d3 = safe_read(3)
    
    print(f"  d0: {d0}")
    print(f"  d10: {d10}")
    print(f"  d3: {d3}")

