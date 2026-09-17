#!/usr/bin/env python3
import time
import threading
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

# Try sending an initial message
print("Sending hello message via d8...")
if write_port(8, "HELLO"):
    print("Message sent!")
    
# Try to receive
print("Listening on d10 for 3 seconds...")
start = time.time()
while time.time() - start < 3:
    msg = read_port(10, timeout=0.5)
    if msg:
        print(f"Received: {msg}")
    time.sleep(0.1)

