#!/usr/bin/env python3
import subprocess
import time

def cmd(p, v):
    subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)

def read(p):
    return subprocess.run(f'timeout 0.3 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True).stdout.strip()

def safe_float(s):
    try: return float(s)
    except: return 0.0

# Let's try reaching specific distances and see if anything changes
target_distances = [0, 50, 100, 200, 500, 1000, 2000, 5000]

for target in target_distances:
    print(f"\nTesting distance={target}")
    
    # Go backwards to reset
    cmd(7, -1)
    while safe_float(read(6)) > 10: time.sleep(0.1)
    cmd(7, 0)
    
    # Now go forward to target
    cmd(7, 1)
    while safe_float(read(6)) < target: time.sleep(0.1)
    cmd(7, 0)
    
    # Check status
    status = read(3)
    dist = read(6)
    d11 = read(11)
    
    print(f"  Status: {status}")
    print(f"  d11: {d11}")
    
    if 'here=1' in status:
        print(f"  *** GOAL FOUND! ***")
        break

print("Done")
