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

# Spiral search: move forward, turn slightly, repeat
print("Starting spiral search...")
cmd(7, 1)  # Start moving

turn_amount = 0
for i in range(100):
    # Every few iterations, adjust steering slightly
    if i % 10 == 0:
        turn_amount += 5
        if turn_amount > 30:
            turn_amount = -30
        cmd(1, turn_amount)
    
    status = read_port(3)
    if 'here=1' in status:
        print(f"GOAL FOUND at iteration {i}!")
        cmd(7, 0)
        break
    
    if i % 10 == 0:
        heading = read_port(4)
        distance = read_port(6)
        print(f"[{i}] turn={turn_amount}, heading={heading}, distance={distance}")
    
    time.sleep(0.5)

cmd(7, 0)
cmd(1, 0)
print("Done")
