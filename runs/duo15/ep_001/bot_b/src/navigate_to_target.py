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

def get_d9():
    try:
        return float(safe_read(9) or 0)
    except:
        return 0

# Target location
targets = [1000, 1500, 2000, 500, 0]

for target in targets:
    print(f"\nNavigating to d9={target}...")
    
    current = get_d9()
    print(f"  Current: {current:.0f}")
    
    if target > current:
        print(f"  Going forward...")
        safe_write(1, "1")
    else:
        print(f"  Going backward...")
        safe_write(1, "-1")
    
    # Move for 20 seconds or until target reached / goal found
    for step in range(100):
        time.sleep(0.2)
        
        current = get_d9()
        status = safe_read(3)
        goal = status.split('goal=')[1][0] if status and 'goal=' in status else '0'
        
        # Check if at target
        if abs(current - target) < 5:
            print(f"  Reached target! d9={current:.0f} goal={goal}")
            break
        
        # Check if overshot
        if target > 0:
            if (target > current + 50) or (current > target + 50):
                if step % 10 == 0:
                    print(f"    d9={current:.0f} (target={target})")
        
        # Check for goal
        if goal == '1':
            print(f"  *** GOAL FOUND at d9={current:.0f}! ***")
            break
    
    safe_write(1, "0")
    time.sleep(0.5)

