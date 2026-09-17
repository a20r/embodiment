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

print("Starting search...")

# Move forward for 10 seconds while monitoring for goal
start = time.time()
cmd(7, 1)  # Start moving

while time.time() - start < 10:
    status = read_port(3)
    heading = read_port(4)
    
    # Parse status
    if 'here=1' in status:
        print(f"GOAL FOUND! Status: {status}")
        break
    
    print(f"Status: {status} | Heading: {heading}")
    time.sleep(1)

cmd(7, 0)  # Stop
print("Done")
