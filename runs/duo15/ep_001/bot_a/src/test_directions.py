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

# Test North from origin
print("Testing North direction (slowly returning to origin first)...")
cmd(7, -1)
while safe_float(read_port(6)) > 5:
    time.sleep(0.2)
cmd(7, 0)

print("Now at origin, moving North...")
cmd(1, 0)  # Try North heading
cmd(7, 1)

for i in range(30):
    dist = safe_float(read_port(6))
    status = read_port(3)
    heading = read_port(4)
    
    if 'here=1' in status:
        print(f"GOAL FOUND at dist={dist}")
        cmd(7, 0)
        exit(0)
    
    if i % 5 == 0:
        print(f"[{i}] dist={dist:.0f}, heading={heading}")
    
    time.sleep(0.5)

cmd(7, 0)
print("No goal in North direction")
