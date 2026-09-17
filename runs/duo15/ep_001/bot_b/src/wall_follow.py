#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.2 cat /dev/robot/d{port_num}",
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

def get_sensors():
    sensors_str = safe_read(2)
    if sensors_str:
        try:
            return [float(x) for x in sensors_str.split(',')]
        except:
            pass
    return []

def get_status():
    status_str = safe_read(3)
    if status_str:
        status = {}
        for part in status_str.split():
            if '=' in part:
                k, v = part.split('=')
                try:
                    status[k] = int(v)
                except:
                    status[k] = v
        return status
    return {}

# Simple wall following strategy
print("Implementing wall-following navigation...")
safe_write(1, "1")  # Start moving

prev_turn_dir = 0  # 0 = straight, 1 = left, -1 = right

for step in range(100):
    sensors = get_sensors()
    status = get_status()
    
    # Check for goal
    if status.get('goal') == 1:
        print(f"\n*** GOAL REACHED at step {step}! ***")
        print(f"Status: {status}")
        break
    
    # Find the direction with maximum clearance
    if len(sensors) >= 16:
        # Assume sensors are arranged circularly
        left_dist = sum(sensors[0:4])  # Left side
        front_dist = sum(sensors[6:10])  # Front
        right_dist = sum(sensors[12:16])  # Right side
        
        print(f"Step {step}: Left={left_dist:.2f} Front={front_dist:.2f} Right={right_dist:.2f} Goal={status.get('goal')}", end="")
        
        # Decision logic
        if front_dist < 1.0:  # Obstacle ahead
            if left_dist > right_dist:
                safe_write(6, "45")
                print(" -> Turn LEFT")
            else:
                safe_write(6, "-45")
                print(" -> Turn RIGHT")
        else:
            print(" -> Go STRAIGHT")
    
    time.sleep(0.5)

safe_write(1, "0")
print("Wall-following complete")

