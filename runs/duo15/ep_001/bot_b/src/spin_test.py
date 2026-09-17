#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.1 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.2
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.2,
            capture_output=True
        )
    except:
        pass

print("Testing spin/orientation...")

# Spin while moving
safe_write(1, "1")

for spin_angle in [0, 90, 180, 270, 360]:
    print(f"\nSpinning to {spin_angle}°...")
    safe_write(6, str(spin_angle))
    
    for step in range(8):
        time.sleep(0.3)
        
        d9 = safe_read(9)
        d4 = safe_read(4)
        status = safe_read(3)
        goal = status.split('goal=')[1][0] if status and 'goal=' in status else '0'
        
        if step % 2 == 0:
            print(f"  d9={d9}, bearing={d4}, goal={goal}")
        
        if goal == '1':
            print(f"  *** GOAL! ***")
            break

safe_write(1, "0")

