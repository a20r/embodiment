#!/usr/bin/env python3
import subprocess
import time
import threading

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.2 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.5
        )
        if result.stdout:
            return result.stdout.strip()
    except:
        pass
    return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.5,
            capture_output=True
        )
    except:
        pass

# Maybe d9 and d11 are communication ports too?
print("Testing all ports as potential communication channels...")
print()

# Try reading from each port with 3 second timeout each
print("Reading from all ports (non-blocking):")
for port in range(12):
    result = safe_read(port)
    if result:
        print(f"d{port:2d}: {result}")

print()

# Try sending on d5 and d7 and listening on other ports
print("Sending 'TEST' on d5...")
safe_write(5, "TEST")
time.sleep(0.5)

print("Checking for responses on all ports:")
for port in [7, 9, 10, 11]:
    result = safe_read(port)
    print(f"d{port}: {result}")

