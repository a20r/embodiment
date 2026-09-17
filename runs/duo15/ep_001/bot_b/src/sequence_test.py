#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.03 cat /dev/robot/d{port_num}",
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

# Sequence to visit: multiples of 500
sequence = [1000, 1500, 2000, 2500, 3000, 3500]

print("Visiting position sequence...")

for target in sequence:
    print(f"\nMoving to d9={target}...")
    
    current_str = safe_read(9)
    current = float(current_str) if current_str else 2500
    
    # Move toward target
    if current < target:
        safe_write(1, "1")
    else:
        safe_write(1, "-1")
    
    # Move for up to 30 seconds or until close
    for _ in range(300):
        d9_str = safe_read(9)
        if not d9_str:
            time.sleep(0.1)
            continue
        
        d9 = float(d9_str)
        
        status = safe_read(3)
        if status and 'goal=1' in status:
            print(f"*** GOAL at sequence position! ***")
            safe_write(1, "0")
            exit(0)
        
        if abs(d9 - target) < 10:
            print(f"  Reached {d9:.0f}")
            break
        
        time.sleep(0.1)
    
    safe_write(1, "0")
    time.sleep(0.3)

print("\nSequence traversal complete")

