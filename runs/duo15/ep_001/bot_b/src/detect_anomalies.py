#!/usr/bin/env python3
import subprocess
import time
import math

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.3 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.5
        )
        if result.stdout:
            return result.stdout.strip()
    except:
        pass
    return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.5,
            capture_output=True
        )
    except:
        pass

def get_sensors():
    sensors_str = safe_read(2)
    if sensors_str:
        try:
            return [float(x) for x in sensors_str.split(',')]
        except:
            pass
    return []

def analyze_sensors(sensors):
    """Analyze sensor data for anomalies"""
    if not sensors:
        return {}
    
    return {
        'min': min(sensors),
        'max': max(sensors),
        'avg': sum(sensors) / len(sensors),
        'std': math.sqrt(sum((x - sum(sensors)/len(sensors))**2 for x in sensors) / len(sensors)),
    }

# Move and sample
print("Sampling sensor data at different locations...")
safe_write(1, "1")

for i in range(15):
    sensors = get_sensors()
    analysis = analyze_sensors(sensors)
    status = safe_read(3)
    
    print(f"Sample {i}: Min={analysis.get('min',0):.3f} Max={analysis.get('max',0):.3f} Avg={analysis.get('avg',0):.3f} Std={analysis.get('std',0):.3f}")
    
    # Check for goal
    if status and 'goal=1' in status:
        print(f"GOAL REACHED! {status}")
        break
    
    time.sleep(0.5)

safe_write(1, "0")
print("Done sampling")

