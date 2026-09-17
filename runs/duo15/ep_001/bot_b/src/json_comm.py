#!/usr/bin/env python3
import subprocess
import time
import json

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

def listen():
    try:
        result = subprocess.run(
            f"timeout 0.1 cat /dev/robot/d10",
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

# Try JSON format messages
messages = [
    json.dumps({"cmd": "ping"}),
    json.dumps({"type": "robot", "status": "searching"}),
    json.dumps({"pos": [0, 0], "looking": True}),
    "JSON ping",
    '{"status":"active"}',
]

print("Sending JSON-format messages...")
for msg in messages:
    print(f"Sending: {msg}")
    safe_write(8, msg)
    
    # Listen for response
    for _ in range(3):
        resp = listen()
        if resp:
            print(f"  Response: {resp}")
        time.sleep(0.1)

