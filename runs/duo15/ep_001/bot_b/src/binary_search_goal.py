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

def get_pos():
    d9 = safe_read(9)
    try:
        return float(d9) if d9 else 0
    except:
        return 0

# Get current position
current = get_pos()
print(f"Current d9 position: {current:.0f}")
print()

# Try going to extreme values
targets = [0, 500, 1000, 1500, 2000, 2500, 3000, 5000, 10000]

for target in targets:
    print(f"\nTrying to reach d9={target}...")
    
    # Pick direction
    if target > current:
        safe_write(1, "1")  # Forward
        direction = "forward"
    else:
        safe_write(1, "-1")  # Backward
        direction = "backward"
    
    # Move for up to 20 seconds
    for step in range(40):
        time.sleep(0.5)
        pos = get_pos()
        status = safe_read(3)
        goal = status.split('goal=')[1][0] if 'goal=' in (status or '') else '?'
        
        if step % 4 == 0:
            print(f"  d9={pos:.0f} (goal={goal})")
        
        # Check if we reached goal
        if status and 'goal=1' in status:
            print(f"*** GOAL FOUND at d9={pos:.0f}! ***")
            break
        
        # Check if we overshot
        if direction == "forward" and pos > target:
            print(f"  Overshot, stopping")
            break
        elif direction == "backward" and pos < target:
            print(f"  Overshot, stopping")
            break
    
    safe_write(1, "0")
    time.sleep(0.3)

