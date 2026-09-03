#!/usr/bin/env python3
import time
import sys
import subprocess

def safe_read(port_num):
    """Safely read from port with timeout"""
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
    """Safely write to port"""
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=1,
            capture_output=True
        )
        return True
    except:
        return False

# Test d5
print("Testing d5 with value 1...")
safe_write(5, "1")
time.sleep(0.5)
status = safe_read(3)
print(f"Status after d5=1: {status}")

# Test d6
print("\nTesting d6 with value 45...")
safe_write(6, "45")
time.sleep(0.5)
bearing = safe_read(4)
print(f"Bearing after d6=45: {bearing}")

# Test moving forward
print("\nTesting d1 with value 1 (move forward)...")
safe_write(1, "1")
time.sleep(1)
status = safe_read(3)
print(f"Status: {status}")

