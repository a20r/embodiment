#!/usr/bin/env python3
import subprocess
import time
import threading

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.05 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.1
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.1,
            capture_output=True
        )
    except:
        pass

print("Testing port read/write combinations...")

# Maybe I need to read from d0 while writing to d1?
print("Experiment 1: Read d0 while writing to d1")
try:
    t1 = threading.Thread(target=lambda: [safe_read(0) for _ in range(10)], daemon=True)
    t1.start()
    time.sleep(0.1)
    for i in range(10):
        safe_write(1, "1" if i % 2 == 0 else "-1")
        time.sleep(0.1)
    t1.join(timeout=2)
except Exception as e:
    print(f"Error: {e}")

status = safe_read(3)
print(f"Result: {status}")
print()

# Try reading d10 while writing d8 rapidly
print("Experiment 2: Read d10 while writing d8 rapidly")
safe_write(1, "0")

goal_found = False
def rapid_broadcast():
    for i in range(100):
        safe_write(8, f"PROTO_{i}")
        time.sleep(0.05)

t2 = threading.Thread(target=rapid_broadcast, daemon=True)
t2.start()

time.sleep(0.1)
for i in range(20):
    msg = safe_read(10)
    if msg:
        print(f"  Got message on iteration {i}: {msg}")
        goal_found = True
        break
    time.sleep(0.1)

t2.join(timeout=2)
status = safe_read(3)
if status and 'goal=1' in status:
    goal_found = True

if goal_found:
    print("  GOAL or MESSAGE FOUND!")
else:
    print("  Nothing found")
print()

# Try writing different patterns to d6 and d1 simultaneously
print("Experiment 3: Complex movement pattern")
safe_write(1, "1")
for i in range(20):
    if i % 3 == 0:
        safe_write(6, str((i * 30) % 360))
    
    status = safe_read(3)
    if status and 'goal=1' in status:
        print("  GOAL FOUND!")
        break
    
    time.sleep(0.1)

safe_write(1, "0")
print()
print("Experiments complete")

