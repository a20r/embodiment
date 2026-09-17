#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.2 cat /dev/robot/d{port_num}",
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

print("Broadcasting position and searching...")
safe_write(1, "1")  # Start moving

for step in range(30):
    d9 = safe_read(9)
    d11 = safe_read(11)
    d4 = safe_read(4)
    status = safe_read(3)
    
    # Broadcast position
    if step % 2 == 0:
        msg = f"POS:{d9},{d11}"
        safe_write(8, msg)
        print(f"[{step}] Sent: {msg}")
    
    # Listen for response
    recv = safe_read(10)
    if recv:
        print(f"    -> RECEIVED: {recv}")
    
    # Check for goal
    if status and 'goal=1' in status:
        print(f"*** GOAL FOUND at position {d9}, {d11}! ***")
        break
    
    time.sleep(0.5)

safe_write(1, "0")
print("Done")

