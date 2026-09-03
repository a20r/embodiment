#!/usr/bin/env python3
import subprocess
import time

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

# Try writing to d9 and d11
print("Testing d9 and d11 as communication ports...")

print("\nWriting 'HELLO' to d9...")
safe_write(9, "HELLO")
time.sleep(0.3)

for port in [10, 11]:
    val = safe_read(port)
    print(f"d{port}: {val}")

print("\nWriting 'WORLD' to d11...")
safe_write(11, "WORLD")
time.sleep(0.3)

for port in [10, 9]:
    val = safe_read(port)
    print(f"d{port}: {val}")

