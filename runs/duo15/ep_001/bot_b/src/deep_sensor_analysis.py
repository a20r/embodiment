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

print("Deep sensor analysis...")
print("Looking for sensor anomalies that might indicate goal location")
print()

# Move slowly and sample sensors frequently
import subprocess
subprocess.run("echo 1 > /dev/robot/d1", shell=True)

anomaly_detected = False
prev_sensors = None

for cycle in range(100):
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
    
    # Look for anomalies
    has_zero = any(x == 0.0 for x in sensors)
    has_negative = any(x < 0 for x in sensors)
    max_val = max(sensors)
    min_val = min(sensors)
    
    # Check for sudden changes
    if prev_sensors:
        max_diff = max(abs(sensors[i] - prev_sensors[i]) for i in range(len(sensors)))
        if max_diff > 0.5:  # Large sudden change
            print(f"ANOMALY at d9={d9_val:.0f}: Large sensor change (diff={max_diff:.3f})")
            anomaly_detected = True
    
    if has_zero:
        print(f"ANOMALY at d9={d9_val:.0f}: Sensor value is exactly 0")
        anomaly_detected = True
    
    if has_negative:
        print(f"ANOMALY at d9={d9_val:.0f}: Negative sensor value detected: {[x for x in sensors if x < 0]}")
    
    if status and 'goal=1' in status:
        print(f"*** GOAL FOUND ***")
        break
    
    prev_sensors = sensors
    time.sleep(0.15)

subprocess.run("echo 0 > /dev/robot/d1", shell=True)
print(f"Analysis complete. Anomalies found: {anomaly_detected}")

