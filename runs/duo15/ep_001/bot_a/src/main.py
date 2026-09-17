#!/usr/bin/env python3
import os
import sys
import time
import threading
import subprocess

# Focus on the transceiver for now - d8 for writing, d10 for reading
# Start a listener thread that will read from d10
def listener():
    """Listen for messages from the other robot"""
    print("[Listener] Starting...")
    # Use timeout to prevent blocking
    while True:
        try:
            result = subprocess.run(['timeout', '5', 'cat', '/dev/robot/d10'], 
                                   capture_output=True, text=True)
            if result.stdout:
                msg = result.stdout.strip()
                print(f"[RX] {msg}")
        except:
            pass
        time.sleep(0.1)

# Start listener in background
listener_thread = threading.Thread(target=listener, daemon=True)
listener_thread.start()

# Send hello message
time.sleep(0.5)
try:
    result = subprocess.run(['timeout', '2', 'bash', '-c', 
                           'echo "HELLO" > /dev/robot/d8'],
                           capture_output=True, text=True)
    print("[TX] HELLO sent")
except Exception as e:
    print(f"[TX] Error: {e}")

# Keep running
time.sleep(2)
print("Done")
