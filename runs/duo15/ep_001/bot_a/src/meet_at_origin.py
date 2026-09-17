#!/usr/bin/env python3
import subprocess
import time
import threading
import queue

def cmd(port, val):
    subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                  capture_output=True, timeout=1)

def read_port(port):
    result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                           capture_output=True, text=True)
    return result.stdout.strip()

def safe_float(s):
    try:
        return float(s) if s else 0.0
    except:
        return 0.0

messages = queue.Queue()

def listener():
    while True:
        msg = read_port(10)
        if msg and len(msg) > 0:
            messages.put(msg)
            print(f"[RX] {msg}")
        time.sleep(0.2)

listener_thread = threading.Thread(target=listener, daemon=True)
listener_thread.start()

# Idea: Reset to origin (distance 0) by going backwards/spinning
print("Attempting to return to origin...")

# Get current odometer
current_dist = safe_float(read_port(6))
print(f"Current distance from origin: {current_dist}")

# Try reversing
print("Moving backwards...")
cmd(7, -1)  # Try negative throttle to go backwards

for i in range(20):
    status = read_port(3)
    dist = safe_float(read_port(6))
    
    print(f"[{i}] distance={dist}")
    
    if 'here=1' in status:
        print("GOAL!")
        break
    
    time.sleep(0.5)

cmd(7, 0)

print("Attempting to find origin/goal with specific heading...")
# Maybe the goal is at a specific bearing from start
# Try going due north (heading 0) or other cardinal directions

