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

print("Moving at 45° bearing continuously...")

safe_write(6, "45")
time.sleep(0.3)
safe_write(1, "1")

for step in range(30):
    time.sleep(0.5)
    
    d9 = safe_read(9)
    d11 = safe_read(11)
    d4 = safe_read(4)
    status = safe_read(3)
    
    x = float(d9) if d9 else 0
    y = float(d11) if d11 else 0
    bearing = float(d4) if d4 else 0
    goal = status.split('goal=')[1][0] if 'goal=' in (status or '') else '?'
    
    if step % 3 == 0:
        print(f"{step:2d}: x={x:7.1f} y={y:.3f} bearing={bearing:6.1f}° goal={goal}")
    
    if status and 'goal=1' in status:
        print(f"*** GOAL! ***")
        break

safe_write(1, "0")

