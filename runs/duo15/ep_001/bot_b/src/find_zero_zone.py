#!/usr/bin/env python3
import subprocess
import time

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

# Start moving and carefully watch for the zero zone
print("Moving slowly and watching for anomalies...")
safe_write(1, "1")

for i in range(30):
    sensors_str = safe_read(2)
    d0 = safe_read(0)
    status = safe_read(3)
    
    # Check if sensors are all zero or empty
    is_zero = False
    if sensors_str:
        sensors = sensors_str.split(',')
        is_zero = all(x.strip() == '0' or x.strip() == '0.000' or x.strip() == '' for x in sensors)
    
    marker = "*** ANOMALY ***" if is_zero else ""
    print(f"{i}: d0={d0} sensors={sensors_str[:40] if sensors_str else 'EMPTY'}... {marker}")
    
    if status and 'goal=1' in status:
        print(f"GOAL! {status}")
        break
    
    time.sleep(0.5)

safe_write(1, "0")
print("Done")

