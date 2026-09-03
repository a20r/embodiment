#!/usr/bin/env python3
import subprocess
import time

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

print("Testing exact positions...")

# Stop
safe_write(1, "0")
time.sleep(0.5)

# Test a grid of positions
targets = [
    2000,
    2500,
    3000,
    1500,
    1000,
]

for target in targets:
    print(f"\nNavigating to d9={target}...")
    
    # Get current
    d9_str = safe_read(9)
    current = float(d9_str) if d9_str else 0
    
    # Decide direction
    if target > current:
        safe_write(1, "1")
    else:
        safe_write(1, "-1")
    
    # Move until close
    while True:
        d9_str = safe_read(9)
        current = float(d9_str) if d9_str else 0
        
        if abs(current - target) < 1:
            print(f"  Reached {current:.0f}")
            break
        
        time.sleep(0.2)
    
    # Stop and check
    safe_write(1, "0")
    
    # Wait and monitor
    print(f"  Monitoring for goal...")
    for wait_step in range(10):
        time.sleep(0.2)
        status = safe_read(3)
        goal = status.split('goal=')[1][0] if status and 'goal=' in status else '0'
        
        if goal == '1':
            print(f"  *** GOAL FOUND! ***")
            break
        
        if wait_step == 9:
            print(f"  No goal at this position")

