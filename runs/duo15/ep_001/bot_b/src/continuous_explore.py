#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    """Read from port"""
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
    """Write to port"""
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=1,
            capture_output=True
        )
    except:
        pass

print("Continuous exploration - moving forward and checking status")
print()

# Move forward continuously and monitor
safe_write(1, "1")
start = time.time()

while time.time() - start < 20:
    d0 = safe_read(0)
    d3 = safe_read(3)
    d4 = safe_read(4)
    
    print(f"d0={d0}, d3={d3}, bearing={d4}")
    
    # Check if we reached goal
    if d3 and 'goal=1' in d3:
        print("GOAL REACHED!")
        break
    
    time.sleep(1)

