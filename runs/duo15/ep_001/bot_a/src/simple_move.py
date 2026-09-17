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

# Just move and check for messages
print("Simple movement test")
cmd(7, 1)

for i in range(30):
    msg = read_port(10)
    if msg:
        print(f"[{i}] RX: {msg}")
    
    status = read_port(3)
    if 'here=1' in status:
        print(f"[{i}] GOAL FOUND!")
        break
    
    if i % 5 == 0:
        print(f"[{i}] Still searching...")
    
    time.sleep(0.5)

cmd(7, 0)
print("Done")
