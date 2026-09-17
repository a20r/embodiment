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

# Broadcast continuously in background
def broadcaster():
    count = 0
    while True:
        count += 1
        msg = f"ROBOT1_ACTIVE:{count}"
        safe_write(8, msg)
        time.sleep(0.5)

# Listen for responses
def listener():
    while True:
        msg = safe_read(10)
        if msg:
            print(f"[RECEIVED] {msg}")
        time.sleep(0.1)

# Monitor status
def monitor():
    while True:
        status = safe_read(3)
        if status and 'goal=1' in status:
            print(f"*** GOAL FOUND! ***")
            print(f"Status: {status}")
        time.sleep(1)

print("Starting broadcast mode...")

t1 = threading.Thread(target=broadcaster, daemon=True)
t2 = threading.Thread(target=listener, daemon=True)
t3 = threading.Thread(target=monitor, daemon=True)

t1.start()
t2.start()
t3.start()

# Move forward while broadcasting
safe_write(1, "1")
time.sleep(120)  # Run for 2 minutes
safe_write(1, "0")

