#!/usr/bin/env python3
import subprocess
import time
import sys

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

# Unbuffered output
sys.stdout = open(sys.stdout.fileno(), 'w', 1)

print("[START] Beginning long-term search")
sys.stdout.flush()

cmd(7, 1)

for iteration in range(3000):
    status = read(3)
    if 'here=1' in status:
        print(f"[GOAL] Found at iteration {iteration}!")
        sys.stdout.flush()
        break
    
    if iteration % 100 == 0:
        dist = read(6)
        heading = read(4)
        print(f"[{iteration}] dist={dist}, heading={heading}")
        sys.stdout.flush()
    
    time.sleep(0.3)

cmd(7, 0)
print("[END] Search complete")
sys.stdout.flush()
