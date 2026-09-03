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

print("Returning to origin (distance=0)...")

cmd(7, -1)  # Go backwards

start_time = time.time()

while time.time() - start_time < 120:  # 2 minute limit
    status = read_port(3)
    dist = safe_float(read_port(6))
    heading = read_port(4)
    
    if dist < 10:  # Near origin
        print(f"Approaching origin! distance={dist:.0f}")
    
    if dist == 0 or 'here=1' in status:
        print(f"*** REACHED ORIGIN OR GOAL! distance={dist}, status={status}")
        cmd(7, 0)
        break
    
    if int(time.time() - start_time) % 10 == 0:
        print(f"[{int(time.time() - start_time)}s] distance={dist:.0f}, heading={heading}")
    
    time.sleep(0.5)

cmd(7, 0)
print("Done")
