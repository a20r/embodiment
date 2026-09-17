#!/usr/bin/env python3
import subprocess, time

def cmd(p,v):
    subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)

def read(p):
    r = subprocess.run(f'timeout 0.15 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1)
    return r.stdout.strip()

print("ULTRA_SEARCH: Maximum efficiency")
sys_start = time.time()
cmd(7, 1)  # Forward

for i in range(500):
    status = read(3)
    rx = read(10)
    
    # Every 100 iterations, change direction slightly
    if i % 100 == 0 and i > 0:
        angle = ((i // 100) * 72) % 360  # Cycle through 0, 72, 144, 216, 288
        cmd(1, angle)
    
    if 'here=1' in status:
        elapsed = time.time() - sys_start
        print(f"GOAL at {i} ({elapsed:.1f}s)")
        break
    
    if rx and len(rx) > 2:
        print(f"RX at {i}: {rx}")
    
    if i % 100 == 0:
        elapsed = time.time() - sys_start
        dist = read(6)
        print(f"{i}: {elapsed:.1f}s, dist={dist}")

cmd(7, 0)
print("COMPLETE")
