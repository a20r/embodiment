#!/usr/bin/env python3
import subprocess
import time

def read_rx():
    try:
        result = subprocess.run(['timeout', '1', 'cat', '/dev/robot/d10'],
                               capture_output=True, text=True)
        return result.stdout.strip()
    except:
        return ""

def send_tx(msg):
    try:
        subprocess.run(['bash', '-c', f'echo "{msg}" > /dev/robot/d8'],
                      capture_output=True, timeout=1)
    except:
        pass

print("Listening for other robot (90 seconds)...")
start = time.time()
msg_count = 0

# Send periodic hellos
send_time = 0

while time.time() - start < 90:
    # Send hello periodically
    if int(time.time() - start) % 10 == 0 and int(time.time() - start) != send_time:
        send_time = int(time.time() - start)
        msg = f"HELLO_FROM_ROBOT_1_AT_{send_time}"
        send_tx(msg)
        print(f"[TX@{send_time}] {msg}")
    
    # Listen
    msg = read_rx()
    if msg and len(msg) > 0:
        msg_count += 1
        elapsed = int(time.time() - start)
        print(f"[RX@{elapsed}] {msg}")
    
    time.sleep(0.5)

print(f"Done. Received {msg_count} messages")
