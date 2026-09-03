#!/usr/bin/env python3
import subprocess
import time
import math

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

print("Attempting geometric proximity search...")
print("Theory: Other robot is at a fixed location, need to get physically close")
print()

# Strategy: Move in a circle around a central position
# If other robot is there, we'll eventually reach communication range

center_d9 = 2500  # Approximate middle of observed range

# Move toward center first
safe_write(1, "1")

for step in range(100):
    d9_str = safe_read(9)
    if not d9_str:
        time.sleep(0.1)
        continue
    
    try:
        d9 = float(d9_str)
    except:
        time.sleep(0.1)
        continue
    
    if abs(d9 - center_d9) < 50:
        print(f"Reached center area at d9={d9:.0f}")
        break
    
    time.sleep(0.1)

# Now perform circular movement with varying turns
print("Performing proximity search pattern...")

safe_write(1, "1")  # Keep moving

for circle_num in range(5):
    print(f"  Circle {circle_num}...")
    
    for angle_step in range(8):
        angle = (angle_step * 45) % 360
        safe_write(6, str(angle))
        
        # Move in this direction for a bit
        for _ in range(20):
            status = safe_read(3)
            if status and 'goal=1' in status:
                print(f"*** GOAL FOUND on circle {circle_num}! ***")
                safe_write(1, "0")
                exit(0)
            
            msg = safe_read(10)
            if msg and msg.strip():
                print(f"*** MESSAGE RECEIVED: {msg} ***")
            
            time.sleep(0.1)

safe_write(1, "0")
print("Geometric search complete")

