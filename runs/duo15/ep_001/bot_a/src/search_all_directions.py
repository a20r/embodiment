#!/usr/bin/env python3
import subprocess
import time

def cmd(port, val):
    try:
        subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                      capture_output=True, timeout=1)
    except:
        pass

def read_port(port, default="0"):
    try:
        result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                               capture_output=True, text=True)
        val = result.stdout.strip()
        return val if val else default
    except:
        return default

def safe_float(s):
    try:
        return float(s)
    except:
        return 0.0

# Try heading in 12 directions (30 degrees apart)
for direction in range(0, 360, 30):
    print(f"\n=== Heading {direction}° ===")
    
    # Set steering to try to achieve this heading
    cmd(1, direction % 360)
    time.sleep(0.2)
    
    # Move forward
    cmd(7, 1)
    start_time = time.time()
    start_dist = safe_float(read_port(6))
    
    while time.time() - start_time < 3:
        status = read_port(3)
        if 'here=1' in status:
            print(f"GOAL FOUND!")
            cmd(7, 0)
            exit(0)
        time.sleep(0.3)
    
    cmd(7, 0)
    end_dist = safe_float(read_port(6))
    heading = read_port(4)
    
    print(f"Traveled {end_dist - start_dist:.0f} units, final heading {heading}°")

print("\nNo goal found in any direction")
