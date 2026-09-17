#!/usr/bin/env python3
import subprocess
import time

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

def get_sensors():
    """Get sensor readings"""
    sensors_str = safe_read(2)
    if sensors_str:
        try:
            return [float(x) for x in sensors_str.split(',')]
        except:
            pass
    return []

print("Analyzing sensor data...")
print()

# Get multiple readings
for i in range(5):
    sensors = get_sensors()
    if sensors:
        print(f"Reading {i}: {len(sensors)} sensors")
        print(f"  Min: {min(sensors):.3f}, Max: {max(sensors):.3f}, Avg: {sum(sensors)/len(sensors):.3f}")
        print(f"  Values: {[f'{x:.3f}' for x in sensors]}")
    time.sleep(0.5)
    print()

