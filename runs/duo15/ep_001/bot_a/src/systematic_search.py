#!/usr/bin/env python3
import subprocess
import time
import math

def cmd(port, val):
    subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                  capture_output=True, timeout=1)

def read_port(port):
    result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                           capture_output=True, text=True)
    return result.stdout.strip()

def parse_status(status_str):
    try:
        parts = status_str.split()
        tick = int(parts[0].split('=')[1])
        goal = int(parts[1].split('=')[1])
        here = int(parts[2].split('=')[1])
        return tick, goal, here
    except:
        return 0, 0, 0

# Try moving in 4 cardinal directions
directions = [
    (0, "North"),
    (90, "East"),
    (180, "South"),
    (270, "West")
]

for target_heading, dir_name in directions:
    print(f"\n=== Trying {dir_name} ({target_heading}°) ===")
    
    # Set heading
    # Need to figure out how to set absolute heading...
    # For now, just try turning to that angle
    cmd(1, target_heading)
    time.sleep(0.5)
    
    # Move forward
    cmd(7, 1)
    time.sleep(5)
    cmd(7, 0)
    
    # Check status
    status = read_port(3)
    heading = read_port(4)
    tick, goal, here = parse_status(status)
    
    print(f"Result: heading={heading}, goal={goal}, here={here}")
    
    if here == 1:
        print(f"*** GOAL FOUND IN {dir_name} ***")
        break

