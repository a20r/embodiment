#!/usr/bin/env python3

import time

# Device paths
LIDAR_DEV = "/dev/robot/lidar"
STATUS_DEV = "/dev/robot/status"
MOTOR_LEFT_DEV = "/dev/robot/motor_left"
MOTOR_RIGHT_DEV = "/dev/robot/motor_right"

def read_device(device_path):
    try:
        with open(device_path, 'r') as f:
            return f.read().strip()
    except:
        return None

def write_motor(left_speed, right_speed):
    try:
        with open(MOTOR_LEFT_DEV, 'w') as f:
            f.write(str(int(left_speed)) + '\n')
        with open(MOTOR_RIGHT_DEV, 'w') as f:
            f.write(str(int(right_speed)) + '\n')
    except:
        pass

def parse_lidar(lidar_str):
    try:
        return [float(x) for x in lidar_str.split(',')]
    except:
        return None

def check_goal():
    status = read_device(STATUS_DEV)
    return status and "goal=1" in status

def smart_navigate():
    """
    Smart maze navigation using reactive control.
    - If path ahead is clear, go forward
    - If blocked, turn right (right-hand wall following)
    - Adapt speed based on distance measurements
    """
    print("Starting smart maze navigation...")
    
    base_speed = 140
    iteration = 0
    max_iterations = 3000
    
    # Beam layout (16 beams, counter-clockwise from front):
    # 0: forward (0°)
    # 4: left-forward (90°)
    # 8: left-rear (180°)
    # 12: right-rear (270°)
    
    while iteration < max_iterations:
        if check_goal():
            print("GOAL REACHED!")
            write_motor(0, 0)
            return True
        
        lidar_str = read_device(LIDAR_DEV)
        if not lidar_str:
            time.sleep(0.03)
            continue
        
        ranges = parse_lidar(lidar_str)
        if not ranges or len(ranges) < 16:
            time.sleep(0.03)
            continue
        
        # Analyze key directions
        forward = ranges[0]      # 0° (straight ahead)
        left45 = ranges[3]       # 45° left
        left90 = ranges[6]       # 90° left
        right45 = ranges[13]     # 45° right
        right90 = ranges[10]     # 90° right (rear-left)
        
        # Replace invalid readings
        for i in range(len(ranges)):
            if ranges[i] < 0:
                ranges[i] = 1.5
        
        forward = max(0.1, ranges[0])
        left45 = max(0.1, ranges[3])
        left90 = max(0.1, ranges[6])
        right45 = max(0.1, ranges[13])
        
        # Decision logic
        left_motor = base_speed
        right_motor = base_speed
        
        # Priority 1: Check for obstacles ahead
        if forward < 0.40:
            # Obstacle ahead - turn right
            left_motor = base_speed
            right_motor = base_speed * 0.3
            action = "Turn right"
        elif forward < 0.50:
            # Close obstacle - turn right more aggressively
            left_motor = base_speed
            right_motor = base_speed * 0.5
            action = "Turn right (close)"
        elif left45 < 0.35:
            # Wall on left, turn right slightly
            left_motor = base_speed + 20
            right_motor = base_speed - 20
            action = "Turn right (wall left)"
        elif right45 < 0.30:
            # Wall on right, turn left
            left_motor = base_speed - 20
            right_motor = base_speed + 20
            action = "Turn left (wall right)"
        else:
            # Open path, go forward
            left_motor = base_speed
            right_motor = base_speed
            action = "Forward"
        
        write_motor(left_motor, right_motor)
        
        iteration += 1
        if iteration % 100 == 0:
            status = read_device(STATUS_DEV)
            print(f"Iter {iteration}: F={forward:.2f}m L={left45:.2f}m R={right45:.2f}m | {action} | {status}")
        
        time.sleep(0.03)
    
    print("Max iterations reached")
    write_motor(0, 0)
    return False

if __name__ == "__main__":
    smart_navigate()
