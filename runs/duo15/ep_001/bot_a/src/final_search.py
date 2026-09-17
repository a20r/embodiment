#!/usr/bin/env python3
import subprocess
import time

def cmd(p,v):
    try:
        subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
    except:
        pass

def read(p):
    try:
        r = subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1)
        return r.stdout.strip()
    except:
        return ""

# Start a continuous search
print("Starting final continuous search...")
cmd(7, 1)  # Start moving

iteration = 0
found = False

while iteration < 1000 and not found:
    # Change direction slowly
    if iteration % 100 == 0 and iteration > 0:
        turn = ((iteration // 100) % 8) * 45  # Cycle through 0, 45, 90, 135, etc
        cmd(1, turn)
        print(f"[{iteration}] Turning to {turn}°")
    
    # Check for goal and messages every iteration
    status = read(3)
    msg = read(10)
    
    if msg and len(msg) > 0:
        print(f"[{iteration}] ***RX***: {msg}")
        found = True
    
    if 'here=1' in status:
        print(f"[{iteration}] ***GOAL REACHED***")
        found = True
    
    iteration += 1
    time.sleep(0.2)

cmd(7, 0)
print(f"Search ended after {iteration} iterations")
