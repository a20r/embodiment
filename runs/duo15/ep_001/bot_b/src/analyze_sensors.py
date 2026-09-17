#!/usr/bin/env python3
import subprocess
import time
import statistics

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.1 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.2
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

# Collect sensor samples
print("Collecting sensor samples...")
samples = []

for _ in range(10):
    sensors_str = safe_read(2)
    if sensors_str:
        try:
            sensors = [float(x) for x in sensors_str.split(',')]
            samples.append(sensors)
        except:
            pass
    time.sleep(0.2)

# Analyze
if samples:
    print(f"Got {len(samples)} samples")
    print(f"Sensors per sample: {len(samples[0])}")
    print()
    
    # Check for anomalies
    for sensor_idx in range(len(samples[0])):
        values = [s[sensor_idx] for s in samples if sensor_idx < len(s)]
        mean = statistics.mean(values)
        stdev = statistics.stdev(values) if len(values) > 1 else 0
        
        print(f"Sensor {sensor_idx:2d}: mean={mean:.3f} stdev={stdev:.3f} values={[f'{v:.2f}' for v in values[:3]]}")

