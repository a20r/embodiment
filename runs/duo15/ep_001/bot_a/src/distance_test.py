#!/usr/bin/env python3
import subprocess
import time

def cmd(p, v):
    subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)

def read(p):
    try:
        result = subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except:
        return ""

def safe_float(s):
    try: return float(s)
    except: return 0.0

print("Testing specific distances...")
cmd(7, 1)

for i in range(30):
    status = read(3)
    dist = safe_float(read(6))
    heading = read(4)
    
    # Check every 500 units
    if i % 10 == 0 or 'here=1' in status:
        print(f"[{i}] dist={dist:.0f}, heading={heading}")
    
    if 'here=1' in status:
        print(f"*** GOAL at distance {dist} ***")
        break
    
    time.sleep(0.5)

cmd(7, 0)
