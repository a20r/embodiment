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

print("Moving in straight line trying to maintain heading...")

# First, establish current heading
heading = float(read_port(4))
print(f"Current heading: {heading}")
target_heading = heading

# Move forward while trying to maintain heading
cmd(7, 1)

start_dist = float(read_port(6))
for i in range(60):
    current_heading = float(read_port(4))
    drift = current_heading - target_heading
    
    # Try to correct drift
    if abs(drift) > 5:
        correction = -drift / 50  # Proportional steering
        cmd(1, correction)
    else:
        cmd(1, 0)
    
    status = read_port(3)
    distance = float(read_port(6))
    
    if i % 10 == 0:
        print(f"[{i}] heading={current_heading:.1f} (target {target_heading:.1f}), dist={distance:.0f} ({distance - start_dist:.0f} traveled)")
    
    if 'here=1' in status:
        print(f"*** GOAL FOUND at {distance:.0f} units! ***")
        break
    
    time.sleep(0.5)

cmd(7, 0)
print("Done")
