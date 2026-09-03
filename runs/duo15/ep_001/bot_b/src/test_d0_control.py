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

# Try writing to d0 while moving
print("Testing d0 as control...")

safe_write(1, "1")
time.sleep(1)

# Try writing 1 to d0 at different positions
for pos_target in [1200, 1300, 1400, 1500]:
    print(f"\nAttempting to write d0=1 when d9~={pos_target}...")
    
    # Wait until we get close to target
    while True:
        d9_str = safe_read(9)
        d9 = float(d9_str) if d9_str else 0
        
        if abs(d9 - pos_target) < 10:
            print(f"  At d9={d9:.0f}, writing d0=1...")
            safe_write(0, "1")
            time.sleep(0.3)
            
            # Check what happened
            status = safe_read(3)
            print(f"    Status: {status}")
            
            if status and 'goal=1' in status:
                print("    *** GOAL! ***")
            break
        
        time.sleep(0.1)

safe_write(1, "0")

