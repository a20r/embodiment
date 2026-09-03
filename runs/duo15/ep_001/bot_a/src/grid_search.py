#!/usr/bin/env python3
import subprocess
import time

def cmd(p, v):
    subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)

def read(p):
    try:
        result = subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except: return ""

def safe_float(s):
    try: return float(s)
    except: return 0.0

print("Grid search pattern...")

# Start from current location and do a grid
for x in range(5):  # Move right/left
    for y in range(5):  # Move forward/back
        # Determine heading
        if x % 2 == 0:
            heading_target = 0  # North
            print(f"[{x},{y}] Moving North")
        else:
            heading_target = 180  # South
            print(f"[{x},{y}] Moving South")
        
        cmd(1, heading_target)
        cmd(7, 1)
        
        # Move in this direction for a bit
        for step in range(10):
            status = read(3)
            if 'here=1' in status:
                print(f"*** GOAL FOUND at ({x},{y},{step}) ***")
                cmd(7, 0)
                exit(0)
            time.sleep(0.5)
        
        cmd(7, 0)
        
        # Turn 90 degrees (by moving East or West based on x)
        if x < 4:
            if x % 2 == 0:
                cmd(1, 90)  # East
            else:
                cmd(1, 270)  # West
            cmd(7, 1)
            for step in range(5):
                time.sleep(0.5)
            cmd(7, 0)

print("Grid search complete")
