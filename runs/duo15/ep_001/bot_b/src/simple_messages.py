#!/usr/bin/env python3
import subprocess
import time

def send_and_listen(message, timeout_sec=2):
    # Send
    try:
        subprocess.run(
            f"echo -n '{message}' > /dev/robot/d8",
            shell=True,
            timeout=0.5,
            capture_output=True
        )
    except:
        pass
    
    print(f"Sent: {repr(message)}")
    
    # Try to listen for response
    start = time.time()
    while time.time() - start < timeout_sec:
        try:
            result = subprocess.run(
                f"timeout 0.1 cat /dev/robot/d10",
                shell=True,
                capture_output=True,
                text=True,
                timeout=0.2
            )
            if result.stdout:
                msg = result.stdout.strip()
                if msg:
                    print(f"  Response: {repr(msg)}")
                    return True
        except:
            pass
        time.sleep(0.1)
    
    print(f"  No response")
    return False

# Try very simple messages
messages = [
    "A",
    "1",
    "X",
    "?",
    "\n",
]

for msg in messages:
    send_and_listen(msg, timeout_sec=1)
    time.sleep(0.5)

