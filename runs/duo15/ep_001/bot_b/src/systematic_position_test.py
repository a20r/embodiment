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

# Test specific positions: multiples of 100 or 500
print("Systematic position testing...")

positions = [1000, 1500, 2000, 2500, 3000, 3500, 4000]

safe_write(1, "0")  # Stop

for target_pos in positions:
    print(f"\nNavigating to d9={target_pos}...")
    
    # Determine direction
    current_str = safe_read(9)
    current = float(current_str) if current_str else 0
    
    if current < target_pos:
        safe_write(1, "1")  # Forward
    else:
        safe_write(1, "-1")  # Backward
    
    # Move until close
    reached = False
    for _ in range(200):
        d9_str = safe_read(9)
        if not d9_str:
            time.sleep(0.05)
            continue
        
        try:
            d9 = float(d9_str)
        except:
            time.sleep(0.05)
            continue
        
        if abs(d9 - target_pos) < 10:
            print(f"  Reached approximately {d9:.0f}")
            reached = True
            break
        
        time.sleep(0.05)
    
    if not reached:
        print(f"  Could not reach {target_pos}")
        continue
    
    # Stop and wait
    safe_write(1, "0")
    time.sleep(0.5)
    
    # Check for goal multiple times
    print(f"  Checking for goal...")
    for wait_iter in range(5):
        status = safe_read(3)
        if status and 'goal=1' in status:
            print(f"  *** GOAL FOUND! ***")
            print(f"  Status: {status}")
            safe_write(1, "0")
            exit(0)
        time.sleep(0.2)

safe_write(1, "0")
print("\nSystematic test complete - no goal found")

