#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.1 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.3
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
            timeout=0.3,
            capture_output=True
        )
    except:
        pass

# Start moving toward 0
print("Moving backward toward d9=0...")
safe_write(1, "-1")

count = 0
while count < 60:
    count += 1
    time.sleep(0.2)
    
    d9_str = safe_read(9)
    status = safe_read(3)
    
    d9 = float(d9_str) if d9_str else 0
    goal = status.split('goal=')[1][0] if status and 'goal=' in status else '?'
    
    if count % 5 == 1:
        print(f"d9={d9:.0f} goal={goal}")
    
    if status and 'goal=1' in status:
        print(f"*** GOAL at d9={d9:.0f}! ***")
        break

safe_write(1, "0")
print("Done")

