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

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.1,
            capture_output=True
        )
    except:
        pass

print("Position-aware protocol experiment...")

safe_write(1, "1")  # Start moving

for cycle in range(100):
    d9 = safe_read(9)
    d11 = safe_read(11)
    d4 = safe_read(4)
    status = safe_read(3)
    
    if not d9:
        time.sleep(0.1)
        continue
    
    try:
        d9_val = int(float(d9))
        d11_val = float(d11)
        d4_val = int(float(d4)) if d4 else 0
    except:
        time.sleep(0.1)
        continue
    
    # Try various message formats with position data
    messages = [
        f"POS:{d9_val}",
        f"{d9_val}:{d11_val}",
        f"AT_{d9_val}_{d4_val}",
        f"LOC:{d9_val}:{d11_val}:{d4_val}",
        f"{d9_val}",
    ]
    
    msg = messages[cycle % len(messages)]
    safe_write(8, msg)
    
    # Check goal
    if status and 'goal=1' in status:
        print(f"*** GOAL at d9={d9_val}! ***")
        break
    
    if cycle % 20 == 0:
        print(f"[{cycle}] Sending: {msg}")
    
    time.sleep(0.1)

safe_write(1, "0")
print("Done")

