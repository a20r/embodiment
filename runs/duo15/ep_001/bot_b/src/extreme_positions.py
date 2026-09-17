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
            timeout=0.2
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.2,
            capture_output=True
        )
    except:
        pass

print("Testing extreme d9 positions...")

#  First go to low value
print("\n1. Moving toward d9=100...")
safe_write(1, "-1")
for i in range(60):
    d9_str = safe_read(9)
    status = safe_read(3)
    
    d9 = float(d9_str) if d9_str else 0
    goal = "1" if status and 'goal=1' in status else "0"
    
    if i % 10 == 0:
        print(f"  d9={d9:.0f} goal={goal}")
    if goal == "1":
        print(f"*** GOAL! ***")
        break
    
    time.sleep(0.2)

safe_write(1, "0")
time.sleep(0.5)

# Now go to high value
print("\n2. Moving toward high d9...")
safe_write(1, "1")
for i in range(60):
    d9_str = safe_read(9)
    status = safe_read(3)
    
    d9 = float(d9_str) if d9_str else 0
    goal = "1" if status and 'goal=1' in status else "0"
    
    if i % 10 == 0:
        print(f"  d9={d9:.0f} goal={goal}")
    if goal == "1":
        print(f"*** GOAL! ***")
        break
    
    time.sleep(0.2)

safe_write(1, "0")
print("\nDone")

