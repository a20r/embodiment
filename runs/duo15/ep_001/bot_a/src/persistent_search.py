#!/usr/bin/env python3
import subprocess
import time
import math

def cmd(port, val):
    try:
        subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                      capture_output=True, timeout=1)
    except:
        pass

def read_port(port):
    try:
        result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                               capture_output=True, text=True)
        return result.stdout.strip()
    except:
        return ""

def safe_float(s):
    try:
        return float(s) if s else 0.0
    except:
        return 0.0

print("[SEARCH] Starting long-term persistent search")
print("[SEARCH] Will search via expanding spiral pattern")

cmd(7, 1)  # Start moving forward

iteration = 0
direction_change = 0
steering_angle = 0

while True:
    iteration += 1
    
    # Change steering slowly to spiral outward
    if iteration % 30 == 0:
        direction_change += 1
        steering_angle = ((direction_change % 24) - 12) * 5  # Sweep from -60 to +60
        cmd(1, steering_angle)
    
    # Check status
    status = read_port(3)
    msg = read_port(10)
    
    if msg and len(msg) > 0 and 'HELLO' in msg.upper():
        print(f"[!!!] FOUND ROBOT! Message: {msg}")
        break
    
    if 'here=1' in status:
        print(f"[!!!] GOAL REACHED!")
        break
    
    # Log periodically
    if iteration % 100 == 0:
        heading = safe_float(read_port(4))
        dist = safe_float(read_port(6))
        print(f"[{iteration}] heading={heading:.0f}, dist={dist:.0f}, turn={steering_angle}")
    
    time.sleep(0.3)
    
    # Safety limit
    if iteration > 5000:
        print("[SEARCH] Reached iteration limit")
        break

cmd(7, 0)
print("[SEARCH] Search ended at iteration", iteration)
