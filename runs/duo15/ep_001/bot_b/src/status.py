#!/usr/bin/env python3
import time
import sys

def read_port(port_num, timeout=1):
    """Try to read from a device port with timeout"""
    try:
        port_path = f"/dev/robot/d{port_num}"
        with open(port_path, 'r') as f:
            line = f.readline()
            return line.strip()
    except Exception as e:
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
        print(f"Error writing to d{port_num}: {e}", file=sys.stderr)
        return False

# Read current status
print("=== Current Status ===")
d3 = read_port(3)
print(f"d3 (status): {d3}")
d4 = read_port(4)
print(f"d4 (bearing?): {d4}")
d0 = read_port(0)
print(f"d0: {d0}")

# Try to understand goal/here values
# Based on d3 output: goal=0, here=0
# This might mean we're not at the goal yet

