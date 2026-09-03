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

print("[PING] Starting ping-based search")

# Continuously send messages
msg_count = 0
for iteration in range(200):
    # Send a message every 5 iterations
    if iteration % 5 == 0:
        cmd(8, f"PING_{iteration}")
        msg_count += 1
    
    # Always move and listen
    cmd(7, 1)
    
    status = read(3)
    rx = read(10)
    
    if rx and len(rx) > 0:
        print(f"[{iteration}] RX: {rx}")
    
    if 'here=1' in status:
        print(f"[{iteration}] GOAL!")
        break
    
    if iteration % 50 == 0:
        print(f"[{iteration}] sent {msg_count} pings")
    
    time.sleep(0.2)

cmd(7, 0)
print("[PING] Search complete")
