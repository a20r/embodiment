#!/usr/bin/env python3
import subprocess
import time
import threading
import queue

def write_tx(msg):
    subprocess.run(['bash', '-c', f'echo "{msg}" > /dev/robot/d8'],
                  capture_output=True, timeout=1)

def read_rx():
    result = subprocess.run(['timeout', '0.3', 'cat', '/dev/robot/d10'],
                           capture_output=True, text=True)
    return result.stdout.strip()

messages = queue.Queue()
found = False

def listener():
    global found
    count = 0
    while not found and count < 100:
        msg = read_rx()
        if msg and len(msg) > 0:
            print(f"[RX] {msg}")
            found = True
            messages.put(msg)
        time.sleep(0.2)
        count += 1

print("Testing different handshake messages...")

# Start listener
threading.Thread(target=listener, daemon=True).start()

messages_to_try = [
    "PING",
    "HELLO",
    "HI",
    "TEST",
    "WHO_IS_THERE",
    "ROBOT_1_HERE",
    "FIND_ME",
    "ARE_YOU_THERE",
    "CONNECT",
    "START",
    "SYNC",
]

for msg in messages_to_try:
    print(f"Sending: {msg}")
    write_tx(msg)
    time.sleep(0.5)

print("Listening for 5 more seconds...")
for _ in range(10):
    time.sleep(0.5)
    msg = read_rx()
    if msg and len(msg) > 0:
        print(f"[RX] {msg}")
        found = True

print("Handshake test complete")
