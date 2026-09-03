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

print("At origin. Testing compass directions...")

directions = [
    (0, "North"),
    (90, "East"),
    (180, "South"),
    (270, "West"),
    (45, "NE"),
    (135, "SE"),
    (225, "SW"),
    (315, "NW")
]

for target, dir_name in directions:
    print(f"\n=== Testing {dir_name} ({target}°) ===")
    
    # Set steering and move
    cmd(1, target % 360)
    time.sleep(0.2)
    cmd(7, 1)  # Move forward
    
    max_dist_in_direction = 0
    for step in range(30):
        dist = safe_float(read_port(6))
        status = read_port(3)
        
        if 'here=1' in status:
            print(f"*** GOAL FOUND after {dist:.0f} units in {dir_name}! ***")
            cmd(7, 0)
            exit(0)
        
        if step % 10 == 0:
            heading = read_port(4)
            print(f"  [{step}] distance={dist:.0f}, heading={heading}")
        
        max_dist_in_direction = dist
        time.sleep(0.3)
    
    # Return to origin
    cmd(7, 0)
    cmd(7, -1)
    
    print(f"  Max distance: {max_dist_in_direction:.0f}")
    
    while safe_float(read_port(6)) > 10:
        time.sleep(0.3)
    
    cmd(7, 0)
    time.sleep(0.5)

print("\nNo goal found in any direction from origin")
