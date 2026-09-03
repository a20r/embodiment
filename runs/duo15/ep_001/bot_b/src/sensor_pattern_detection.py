#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.05 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.1
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

print("Sensor pattern detection...")

# Move and collect sensor data
subprocess.run("echo 1 > /dev/robot/d1", shell=True)

sensor_history = []

for cycle in range(50):
    sensors_str = safe_read(2)
    d9 = safe_read(9)
    status = safe_read(3)
    
    if not sensors_str or not d9:
        time.sleep(0.1)
        continue
    
    try:
        sensors = [float(x) for x in sensors_str.split(',')]
        d9_val = float(d9)
    except:
        time.sleep(0.1)
        continue
    
    sensor_history.append((d9_val, sensors))
    
    # Check for goal
    if status and 'goal=1' in status:
        print(f"GOAL FOUND!")
        break
    
    # Look for specific patterns
    # Pattern 1: All sensors same value (symmetric environment)
    if all(abs(s - sensors[0]) < 0.01 for s in sensors):
        print(f"SYMMETRY DETECTED at d9={d9_val:.0f}")
    
    # Pattern 2: Sensors forming specific sequence
    sorted_sensors = sorted(sensors)
    if sorted_sensors[0] == sorted_sensors[1] == sorted_sensors[2]:
        print(f"TRIPLET at d9={d9_val:.0f}: {sorted_sensors[:3]}")
    
    # Pattern 3: Specific sensor reading
    if any(abs(s - 2.0) < 0.01 for s in sensors):  # Sensor reads 2.0 exactly
        print(f"EXACT 2.0 at d9={d9_val:.0f}")
    
    if cycle % 10 == 0:
        print(f"  [{cycle}] Scanning at d9={d9_val:.0f}")
    
    time.sleep(0.1)

subprocess.run("echo 0 > /dev/robot/d1", shell=True)

print(f"\nCollected {len(sensor_history)} samples")
if len(sensor_history) > 0:
    print(f"Position range: {sensor_history[0][0]:.0f} to {sensor_history[-1][0]:.0f}")

