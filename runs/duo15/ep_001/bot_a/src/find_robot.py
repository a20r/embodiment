#!/usr/bin/env python3
import subprocess
import time
import threading
import queue
import math

def cmd(port, val):
    try:
        subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                      capture_output=True, timeout=1)
    except:
        pass

def read_port(port):
    try:
        result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                               capture_output=True, text=True)
        return result.stdout.strip()
    except:
        return ""

def safe_float(s):
    try:
        return float(s) if s else 0.0
    except:
        return 0.0

messages = queue.Queue()
found_robot = False

def listener():
    global found_robot
    while not found_robot:
        msg = read_port(10)
        if msg and len(msg) > 0 and "HELLO" in msg.upper():
            messages.put(msg)
            print(f"[!!!] RECEIVED MESSAGE: {msg}")
            found_robot = True
            return
        time.sleep(0.2)

# Start listener
listener_thread = threading.Thread(target=listener, daemon=True)
listener_thread.start()

print("Searching for the other robot...")
print("Strategy: Moving in expanding square pattern")

# Start moving in expanding squares
cmd(7, 1)  # Start forward

side_length = 1  # number of 10-second moves per side
side = 0
direction = 0  # 0=forward, 1=turn right, 2=continue

for i in range(200):
    if found_robot:
        print(f"Found other robot at iteration {i}!")
        cmd(7, 0)
        break
    
    status = read_port(3)
    if 'here=1' in status:
        print(f"Goal reached at iteration {i}!")
        cmd(7, 0)
        break
    
    heading = read_port(4)
    distance = read_port(6)
    
    if i % 10 == 0:
        print(f"[{i}] heading={heading}°, dist={distance}, searching...")
    
    # Check messages
    try:
        while True:
            msg = messages.get_nowait()
            print(f"[MSG] {msg}")
    except queue.Empty:
        pass
    
    time.sleep(0.5)

cmd(7, 0)
print("Search complete")
