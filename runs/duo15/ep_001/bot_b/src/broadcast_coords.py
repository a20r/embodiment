#!/usr/bin/env python3
import subprocess
import time
import threading

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

print("Broadcasting coordinates continuously...")

# Move forward
safe_write(1, "1")

for i in range(60):
    d9 = safe_read(9)
    d11 = safe_read(11)
    status = safe_read(3)
    
    # Broadcast coordinates
    if d9 and d11:
        msg = f"R1:{d9},{d11}"
        safe_write(8, msg)
    
    # Check for response
    recv = safe_read(10)
    if recv and recv.strip():
        print(f"[{i}] Received: {recv}")
    
    # Check for goal
    if status and 'goal=1' in status:
        print(f"GOAL FOUND at step {i}!")
        break
    
    if i % 10 == 0:
        print(f"[{i}] Sent: {msg if d9 and d11 else 'N/A'}")
    
    time.sleep(1)

safe_write(1, "0")
print("Done")

