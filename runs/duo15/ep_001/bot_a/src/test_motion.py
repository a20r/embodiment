#!/usr/bin/env python3
import subprocess
import time

def get_info():
    d0 = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d0'],
                       capture_output=True, text=True).stdout.strip()
    d3 = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d3'],
                       capture_output=True, text=True).stdout.strip()
    d4 = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d4'],
                       capture_output=True, text=True).stdout.strip()
    d5 = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d5'],
                       capture_output=True, text=True).stdout.strip()
    d6 = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d6'],
                       capture_output=True, text=True).stdout.strip()
    return f"d0={d0} d5={d5} d6={d6} heading={d4}"

def write_cmd(port, value):
    subprocess.run(['bash', '-c', f'echo "{value}" > /dev/robot/d{port}'],
                  capture_output=True, timeout=2)

print("=== Test moving forward ===")
print("Before:", get_info())
write_cmd(7, "1")  # Try d7 for forward
time.sleep(0.5)
print("After d7=1:", get_info())

write_cmd(7, "0")
time.sleep(0.3)
print("After d7=0:", get_info())

print("\n=== Test steering ===")
write_cmd(7, "1")
write_cmd(1, "1")
time.sleep(0.5)
print("d7=1, d1=1:", get_info())

write_cmd(1, "-1")
time.sleep(0.5)
print("d7=1, d1=-1:", get_info())

