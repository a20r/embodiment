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
            timeout=0.15
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.15,
            capture_output=True
        )
    except:
        pass

print("Navigating to d9=2000 and checking for goal...")

# Get current position
d9_str = safe_read(9)
current = float(d9_str) if d9_str else 0
print(f"Current d9: {current:.0f}")

# Determine direction
if current > 2000:
    safe_write(1, "-1")
    print("Moving backward...")
else:
    safe_write(1, "1")
    print("Moving forward...")

# Move until at 2000
count = 0
while count < 200:
    d9_str = safe_read(9)
    if not d9_str:
        time.sleep(0.1)
        continue
    
    try:
        d9 = float(d9_str)
    except:
        time.sleep(0.1)
        continue
    
    count += 1
    
    # Check if close to target
    if 1995 < d9 < 2005:
        print(f"Close to target: d9={d9:.0f}")
        safe_write(1, "0")
        break
    
    if count % 20 == 1:
        print(f"  [{count}] d9={d9:.0f}")
    
    time.sleep(0.1)

print("\nWaiting at d9~2000 for goal trigger...")
time.sleep(1)

# Monitor for goal
for i in range(30):
    status = safe_read(3)
    d9 = safe_read(9)
    goal = status.split('goal=')[1][0] if status and 'goal=' in status else '?'
    
    print(f"[{i}] d9={d9} goal={goal}")
    
    if status and 'goal=1' in status:
        print("*** GOAL FOUND! ***")
        break
    
    # Also broadcast
    safe_write(8, f"WAITING_AT_2000_{i}")
    
    time.sleep(1)

