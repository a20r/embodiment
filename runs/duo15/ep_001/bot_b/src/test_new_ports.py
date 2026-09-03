#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.5 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=1
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
            timeout=1,
            capture_output=True
        )
    except:
        pass

# Try reading from ports that haven't given output
print("Testing read from d7...")
d7 = safe_read(7)
print(f"d7: {d7}")

# Let me try writing something to understand what ports do
print("\nTrying d5 with different values...")
for val in ["0", "1", "10", "100"]:
    safe_write(5, val)
    time.sleep(0.3)
    status = safe_read(3)
    print(f"d5={val}: {status}")

print("\nTrying d7 with different values...")
for val in ["0", "1", "-1", "5"]:
    safe_write(7, val)
    time.sleep(0.3)
    status = safe_read(3)
    d9 = safe_read(9)
    print(f"d7={val}: status={status}, d9={d9}")

