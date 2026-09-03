#!/usr/bin/env python3
import subprocess

def get_sensors():
    result = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d2'],
                           capture_output=True, text=True)
    try:
        values = [float(x) for x in result.stdout.strip().split(',')]
        return values
    except:
        return []

sensors = get_sensors()
print(f"Found {len(sensors)} sensors")
for i, val in enumerate(sensors):
    if val > 0.1:  # Highlight non-zero readings
        print(f"  Sensor {i}: {val:.3f} <--")
    else:
        print(f"  Sensor {i}: {val:.3f}")
