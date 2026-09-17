#!/usr/bin/env python3
import time
import subprocess
import json

def safe_read(port_num):
    """Safely read from port"""
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

def get_status():
    """Parse status"""
    status_str = safe_read(3)
    if status_str:
        status = {}
        for part in status_str.split():
            if '=' in part:
                k, v = part.split('=')
                try:
                    status[k] = int(v)
                except:
                    status[k] = v
        return status
    return {}

def get_bearing():
    """Get bearing"""
    bearing_str = safe_read(4)
    if bearing_str:
        try:
            return float(bearing_str)
        except:
            pass
    return None

# Strategy: try moving in different directions to see if goal changes
print("Starting exploration to find goal...")
print()

directions = [0, 90, 180, 270]
for direction in directions:
    print(f"Turning to bearing {direction}...")
    safe_write(6, str(direction))
    time.sleep(0.5)
    
    bearing = get_bearing()
    status = get_status()
    print(f"  Current bearing: {bearing}")
    print(f"  Status: {status}")
    
    # Try moving forward in this direction
    print(f"  Moving forward...")
    safe_write(1, "1")
    time.sleep(2)
    
    status = get_status()
    bearing = get_bearing()
    print(f"  After moving - bearing: {bearing}, status: {status}")
    print()

