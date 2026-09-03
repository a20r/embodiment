#!/usr/bin/env python3
import subprocess
import time
import threading
import queue

def cmd(port, val):
    try:
        subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'], 
                      capture_output=True, timeout=1)
    except:
        pass

def read_port(port, default=""):
    try:
        result = subprocess.run(['timeout', '0.3', 'cat', f'/dev/robot/d{port}'],
                               capture_output=True, text=True)
        return result.stdout.strip() or default
    except:
        return default

def safe_float(s):
    try:
        return float(s)
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

# Start listener thread
listener_thread = threading.Thread(target=listener, daemon=True)
listener_thread.start()

print("=== Robot 1 Coordination System ===")
print("Sending HELLO...")
cmd(8, "HELLO_I_AM_ROBOT_1")

# Wait for acknowledgment
time.sleep(2)

# Broadcast my info
my_id = 1
initial_dist = safe_float(read_port(6))
print(f"My initial distance: {initial_dist}")

# Start moving and periodically broadcast
cmd(7, 1)

for i in range(30):
    status = read_port(3)
    heading_str = read_port(4)
    distance_str = read_port(6)
    heading = safe_float(heading_str)
    distance = safe_float(distance_str)
    
    if i % 5 == 0:
        msg = f"ROBOT_1:h={heading:.0f},d={distance:.0f}"
        cmd(8, msg)
        print(f"[TX] {msg}")
    
    if 'here=1' in status:
        print("*** GOAL REACHED ***")
        break
    
    # Check for messages
    try:
        while True:
            msg = messages.get_nowait()
    except queue.Empty:
        pass
    
    time.sleep(1)

cmd(7, 0)
print("Coordination attempt finished")
