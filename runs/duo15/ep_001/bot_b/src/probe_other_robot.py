#!/usr/bin/env python3
import subprocess
import time
import threading

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=1,
            capture_output=True
        )
    except:
        pass

def listen_for_response(duration=5):
    """Listen for incoming messages"""
    end_time = time.time() + duration
    responses = []
    
    while time.time() < end_time:
        try:
            result = subprocess.run(
                f"timeout 0.1 cat /dev/robot/d10",
                shell=True,
                capture_output=True,
                text=True,
                timeout=0.5
            )
            if result.stdout:
                msg = result.stdout.strip()
                if msg:
                    responses.append(msg)
                    print(f"Received: {msg}")
        except:
            pass
        time.sleep(0.05)
    
    return responses

# Start a listener thread
listener_thread = threading.Thread(target=listen_for_response, args=(15,), daemon=True)
listener_thread.start()

# Send various message formats
messages = [
    "PING",
    "POSITION?",
    "GOAL?",
    "READY",
    "SEARCHING",
    "123",
    "LOC",
]

print("Sending probe messages...")
for msg in messages:
    safe_write(8, msg)
    print(f"Sent: {msg}")
    time.sleep(1)

print("Waiting for responses...")
listener_thread.join(timeout=15)
print("Done")

