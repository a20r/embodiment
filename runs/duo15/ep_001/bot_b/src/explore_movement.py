#!/usr/bin/env python3
import time
import sys

def read_port(port_num, timeout=1):
    """Read from a device port"""
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
    except:
        return False

# Test different combinations
print("Testing various movement commands...")

# Try d7
print("\n1. Testing d7...")
write_port(7, "1")
time.sleep(0.5)
print(f"d9: {read_port(9)}")

# Try negative d1
print("\n2. Testing negative d1...")
write_port(1, "-1")
time.sleep(0.5)
print(f"d6: {read_port(6)}")
print(f"d9: {read_port(9)}")

# Try d5
print("\n3. Testing d5...")
write_port(5, "1")
time.sleep(0.5)
print(f"Status: {read_port(3)}")

# Get full status
print("\n=== Full Status ===")
for port in [0, 2, 3, 4, 5, 6, 7, 9, 11]:
    val = read_port(port)
    if val:
        print(f"d{port}: {val}")

