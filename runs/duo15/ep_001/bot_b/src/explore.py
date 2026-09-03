#!/usr/bin/env python3
import time
import threading
import os
import sys

def read_port(port_num, timeout=2):
    """Try to read from a device port with timeout"""
    try:
        port_path = f"/dev/robot/d{port_num}"
        with open(port_path, 'r') as f:
            # Set non-blocking if possible
            import select
            ready = select.select([f], [], [], timeout)
            if ready[0]:
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

# Test some ports
print("Testing device ports...")

# Try reading from various ports to understand the robot's sensors/state
for i in range(12):
    print(f"\nTrying to read from d{i}...")
    result = read_port(i, timeout=0.5)
    if result:
        print(f"  d{i}: {result}")

