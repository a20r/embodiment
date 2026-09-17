#!/usr/bin/env python3
import subprocess
import time
import threading

def safe_write(port_num, message):
    """Write to port"""
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=1,
            capture_output=True
        )
        return True
    except:
        return False

def safe_read_noblock(port_num):
    """Non-blocking read"""
    try:
        result = subprocess.run(
            f"timeout 0.1 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=1
        )
        if result.stdout:
            return result.stdout.strip()
    except:
        pass
    return None

# Continuously broadcast
print("Broadcasting messages...")
messages = [
    "HELLO",
    "I AM HERE",
    "ROBOT_ONLINE",
    "WHO_ARE_YOU",
    "I_SEEK_GOAL",
    "STATUS",
]

msg_idx = 0
start = time.time()
while time.time() - start < 10:
    msg = messages[msg_idx % len(messages)]
    safe_write(8, msg)
    print(f"Sent: {msg}")
    
    # Try to read responses
    for _ in range(3):
        response = safe_read_noblock(10)
        if response:
            print(f"  -> RECEIVED: {response}")
        time.sleep(0.1)
    
    msg_idx += 1
    time.sleep(0.5)

print("Done broadcasting")

