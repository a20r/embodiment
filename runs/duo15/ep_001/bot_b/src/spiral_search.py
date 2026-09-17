#!/usr/bin/env python3
"""
Try spiral movement pattern while broadcasting
"""
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.05 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.1
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.1,
            capture_output=True
        )
    except:
        pass

print("Spiral search with continuous broadcasting...")

# Start at current position and spiral outward
safe_write(1, "1")  # Start moving

angle = 0
speed_cycles = 0

for cycle in range(50):
    # Every 10 cycles, increase spiral radius by turning
    if cycle % 10 == 0:
        angle = (angle + 45) % 360
        safe_write(6, str(angle))
        print(f"Cycle {cycle}: turned to {angle}°")
    
    # Send position broadcast
    d9 = safe_read(9)
    d11 = safe_read(11)
    msg = f"SPIRAL:{d9}:{d11}:{cycle}"
    safe_write(8, msg)
    
    # Check for goal
    status = safe_read(3)
    if status and 'goal=1' in status:
        print(f"*** GOAL FOUND at cycle {cycle}! ***")
        print(f"Position: d9={d9}, d11={d11}")
        break
    
    if cycle % 10 == 0:
        print(f"  [{cycle}] d9={d9}, broadcasting...")
    
    time.sleep(0.2)

safe_write(1, "0")
print("Spiral search complete")

