#!/usr/bin/env python3
import subprocess
import threading
import time
import queue

message_queue = queue.Queue()

def continuous_sender():
    """Send messages continuously"""
    counter = 0
    while True:
        counter += 1
        try:
            subprocess.run(
                f"echo 'ROBOT1_MSG_{counter}' > /dev/robot/d8",
                shell=True,
                timeout=0.05,
                capture_output=True
            )
        except:
            pass
        time.sleep(0.1)

def continuous_receiver():
    """Receive messages continuously"""
    while True:
        try:
            result = subprocess.run(
                "timeout 0.2 cat /dev/robot/d10",
                shell=True,
                capture_output=True,
                text=True,
                timeout=0.3
            )
            if result.stdout and result.stdout.strip():
                message_queue.put(result.stdout.strip())
        except:
            pass
        time.sleep(0.05)

def goal_monitor():
    """Monitor for goal"""
    while True:
        try:
            result = subprocess.run(
                "timeout 0.05 cat /dev/robot/d3",
                shell=True,
                capture_output=True,
                text=True,
                timeout=0.1
            )
            if result.stdout and 'goal=1' in result.stdout:
                return True
        except:
            pass
        time.sleep(0.2)

print("Starting simultaneous continuous communication...")

# Start threads
sender_thread = threading.Thread(target=continuous_sender, daemon=True)
receiver_thread = threading.Thread(target=continuous_receiver, daemon=True)

sender_thread.start()
receiver_thread.start()

# Monitor for 60 seconds
print("Monitoring for goal or messages...")
start = time.time()

while time.time() - start < 60:
    # Check for received messages
    try:
        while True:
            msg = message_queue.get_nowait()
            print(f"[RECEIVED] {msg}")
    except queue.Empty:
        pass
    
    # Check for goal
    try:
        result = subprocess.run(
            "timeout 0.05 cat /dev/robot/d3",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.1
        )
        if result.stdout and 'goal=1' in result.stdout:
            print(f"*** GOAL FOUND ***")
            print(result.stdout)
            break
    except:
        pass
    
    time.sleep(0.5)

print("Monitoring complete")

