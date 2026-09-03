#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.3 cat /dev/robot/d{port_num}",
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

def listen_once():
    """Quick listen for messages"""
    return safe_read(10)

# Systematic exploration: move in different directions
directions = [0, 45, 90, 135, 180, 225, 270, 315]

print("Systematic exploration - moving in each direction")
print()

for direction in directions:
    print(f"\n=== Direction {direction} degrees ===")
    
    # Stop current movement
    safe_write(1, "0")
    time.sleep(0.3)
    
    # Set heading
    safe_write(6, str(direction))
    time.sleep(0.3)
    
    # Move forward
    safe_write(1, "1")
    
    # Move for 5 seconds, listening for messages
    end_time = time.time() + 5
    msg_count = 0
    
    while time.time() < end_time:
        msg = listen_once()
        if msg:
            print(f"  RECEIVED: {msg}")
            msg_count += 1
        
        # Check status occasionally
        if int((time.time() - end_time + 5) * 2) % 2 == 0:
            status = safe_read(3)
            if 'goal=1' in (status or ''):
                print(f"  GOAL REACHED! Status: {status}")
                break
        
        time.sleep(0.2)
    
    print(f"  No messages received in this direction")

# Stop
safe_write(1, "0")
print("\nExploration complete")

