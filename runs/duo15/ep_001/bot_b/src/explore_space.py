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
    d11 = safe_read(11)
    try:
        return (float(d9) if d9 else 0, float(d11) if d11 else 0)
    except:
        return (0, 0)

print("Exploring space to find goal...")
print()

# Directions to explore
directions = [0, 90, 180, 270, 45, 135, 225, 315]

for direction in directions:
    print(f"\n=== Direction {direction}° ===")
    
    # Stop
    safe_write(1, "0")
    time.sleep(0.3)
    
    # Turn
    safe_write(6, str(direction))
    time.sleep(0.3)
    
    # Move and sample
    safe_write(1, "1")
    
    for step in range(5):
        time.sleep(1)
        
        x, y = get_pos()
        status = safe_read(3)
        goal = status.split('goal=')[1][0] if 'goal=' in (status or '') else '?'
        
        print(f"  Step {step}: ({x:.1f}, {y:.3f}) goal={goal}")
        
        if status and 'goal=1' in status:
            print(f"*** GOAL FOUND! ***")
            break

safe_write(1, "0")
print("\nExploration complete")

