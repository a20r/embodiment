#!/usr/bin/env python3
import subprocess
import time

def read_port(port):
    result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                           capture_output=True, text=True)
    return result.stdout.strip()

def cmd(port, val):
    subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                  capture_output=True, timeout=1)

print("Monitoring d3 while moving...")

cmd(7, 1)

last_status = ""
for i in range(60):
    status = read_port(3)
    if status != last_status:
        print(f"[{i}] {status}")
        last_status = status
    time.sleep(0.5)

cmd(7, 0)
