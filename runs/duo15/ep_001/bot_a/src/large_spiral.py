#!/usr/bin/env python3
import subprocess
import time

def cmd(port, val):
    subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                  capture_output=True, timeout=1)

def read_port(port):
    result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                           capture_output=True, text=True)
    return result.stdout.strip()

def safe_float(s):
    try:
        return float(s) if s else 0.0
    except:
        return 0.0

print("Large expanding spiral search...")
cmd(7, 1)  # Start moving

turn_angle = 0
steps = 0

for i in range(500):
    steps += 1
    
    # Slowly increase turning over time
    if i % 20 == 0:
        turn_angle += 10
        if turn_angle > 45:
            turn_angle = -45
        cmd(1, turn_angle)
    
    # Check for goal or messages
    status = read_port(3)
    msg = read_port(10)
    
    if msg and len(msg) > 0:
        print(f"[{i}] **RX** {msg}")
    
    if 'here=1' in status:
        print(f"[{i}] **GOAL FOUND**")
        cmd(7, 0)
        break
    
    if i % 50 == 0:
        dist = read_port(6)
        heading = read_port(4)
        print(f"[{i}] turn={turn_angle}, dist={dist}, heading={heading}")
    
    time.sleep(0.3)

cmd(7, 0)
print(f"Spiral complete, traveled {steps} steps")
