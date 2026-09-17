#!/usr/bin/env python3
import time
import sys

def read_port(port_num, timeout=1):
    """Try to read from a device port"""
    try:
        port_path = f"/dev/robot/d{port_num}"
        with open(port_path, 'r') as f:
            line = f.readline()
            return line.strip()
    except:
        pass
    return None

def write_port(port_num, message):
    """Write to a device port"""
    try:
        port_path = f"/dev/robot/d{port_num}"
        with open(port_path, 'w') as f:
            f.write(message + '\n')
            f.flush()
        return True
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return False

# Test different write ports to understand control
print("Testing control ports...")

# Try d1 - maybe forward/backward?
print("\nTrying d1 with value 1...")
write_port(1, "1")
time.sleep(1)
status = read_port(3)
print(f"Status: {status}")

# Check current state
print("\nCurrent state on all readable ports:")
for i in [0, 2, 3, 4, 5, 6, 9, 11]:
    val = read_port(i)
    if val:
        print(f"d{i}: {val}")

