#!/usr/bin/env python3
import subprocess
import time

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

# Key positions to try: powers of 2, round numbers, special values
key_positions = [
    512, 1024, 2048, 4096,  # Powers of 2
    1111, 2222, 3333, 4444,  # Repeating digits
    1337, 2048, 3141,  # Common special numbers
    1500, 2000, 2500, 3000, 3500,  # Round numbers
]

print("Testing key positions with long waits...")

for target_pos in key_positions:
    print(f"\n=== Testing d9={target_pos} ===")
    
    # Get current
    current_str = safe_read(9)
    current = float(current_str) if current_str else 0
    
    # Move toward target
    if current < target_pos:
        safe_write(1, "1")
        direction = "forward"
    else:
        safe_write(1, "-1")
        direction = "backward"
    
    print(f"Moving {direction} from {current:.0f}...")
    
    # Move for up to 30 seconds
    for _ in range(300):
        d9_str = safe_read(9)
        if not d9_str:
            time.sleep(0.1)
            continue
        
        d9 = float(d9_str)
        
        if abs(d9 - target_pos) < 5:
            print(f"Reached {d9:.0f}")
            break
        
        time.sleep(0.1)
    
    # Stop
    safe_write(1, "0")
    
    # Wait 10 seconds at this position
    print("Waiting 10 seconds...")
    for wait_sec in range(10):
        time.sleep(1)
        status = safe_read(3)
        
        if status and 'goal=1' in status:
            print(f"*** GOAL FOUND at d9={target_pos}! ***")
            print(f"Status: {status}")
            exit(0)
        
        if wait_sec == 4:
            print("  5 seconds elapsed...")

print("\nNo goal found at any key position")

