#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.04 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.08
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.08,
            capture_output=True
        )
    except:
        pass

print("Moving backward to find absolute minimum d9...")

safe_write(1, "-1")  # Full backward

prev_d9 = 9999
stuck_count = 0

for cycle in range(500):
    d9_str = safe_read(9)
    status = safe_read(3)
    
    if not d9_str:
        time.sleep(0.05)
        continue
    
    try:
        d9 = float(d9_str)
    except:
        time.sleep(0.05)
        continue
    
    # Check for goal
    if status and 'goal=1' in status:
        print(f"*** GOAL at minimum position d9={d9:.0f}! ***")
        break
    
    # Check if stuck (not moving)
    if abs(d9 - prev_d9) < 0.1:
        stuck_count += 1
    else:
        stuck_count = 0
    
    prev_d9 = d9
    
    if cycle % 50 == 0:
        print(f"[{cycle}] d9={d9:.0f} stuck={stuck_count}")
    
    # If stuck for a while, might have hit minimum
    if stuck_count > 10:
        print(f"Hit minimum at d9={d9:.0f}")
        safe_write(1, "0")
        
        # Wait here
        print("Waiting at minimum position...")
        for wait in range(10):
            status = safe_read(3)
            if status and 'goal=1' in status:
                print(f"*** GOAL triggered at minimum! ***")
                break
            time.sleep(0.5)
        
        break
    
    time.sleep(0.05)

safe_write(1, "0")
print("Movement complete")

