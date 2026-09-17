#!/usr/bin/env python3
import subprocess
import time

def get_status():
    result = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d3'],
                           capture_output=True, text=True)
    return result.stdout.strip()

def get_heading():
    result = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d4'],
                           capture_output=True, text=True)
    return result.stdout.strip()

def write_command(port, value):
    subprocess.run(['bash', '-c', f'echo "{value}" > /dev/robot/d{port}'],
                  capture_output=True, timeout=2)

print("Initial status:", get_status())
print("Initial heading:", get_heading())

# Try moving with d1
print("\n--- Writing 1 to d1 ---")
write_command(1, "1")
time.sleep(0.3)
print("Status:", get_status())
print("Heading:", get_heading())

# Try moving with different value
print("\n--- Writing 0 to d1 ---")
write_command(1, "0")
time.sleep(0.3)
print("Status:", get_status())
print("Heading:", get_heading())

# Try negative
print("\n--- Writing -1 to d1 ---")
write_command(1, "-1")
time.sleep(0.3)
print("Status:", get_status())
print("Heading:", get_heading())

