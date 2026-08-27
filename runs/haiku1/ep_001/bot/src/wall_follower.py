#!/usr/bin/env python3

import time
import subprocess

# Device paths
LIDAR_DEV = "/dev/robot/lidar"
STATUS_DEV = "/dev/robot/status"
MOTOR_LEFT_DEV = "/dev/robot/motor_left"
MOTOR_RIGHT_DEV = "/dev/robot/motor_right"

def read_device(device_path):
    """Read a device file and return the value"""
    try:
        with open(device_path, 'r') as f:
            return f.read().strip()
    except Exception as e:
        return None

def write_motor(left_speed, right_speed):
    """Write motor commands"""
    try:
        with open(MOTOR_LEFT_DEV, 'w') as f:
            f.write(str(left_speed) + '\n')
        with open(MOTOR_RIGHT_DEV, 'w') as f:
            f.write(str(right_speed) + '\n')
    except Exception as e:
        pass

def parse_lidar(lidar_str):
    """Parse lidar string into list of ranges"""
    try:
        return [float(x) for x in lidar_str.split(',')]
    except:
        return None

def check_goal():
    """Check if we've reached the goal"""
    status = read_device(STATUS_DEV)
    return status and "goal=1" in status

def wall_follow_left():
    """
    Wall following on the left side.
    Keep the left wall at a constant distance by adjusting right motor.
    """
    print("Starting left wall following...")
    
    base_speed = 120
    target_distance = 0.35
    tolerance = 0.05
    max_iterations = 2000
    iteration = 0
    stuck_count = 0
    last_front = 0.5
    
    while iteration < max_iterations:
        if check_goal():
            print("GOAL REACHED!")
            write_motor(0, 0)
            return True
        
        lidar_str = read_device(LIDAR_DEV)
        if not lidar_str:
            time.sleep(0.05)
            continue
        
        ranges = parse_lidar(lidar_str)
        if not ranges or len(ranges) < 16:
            time.sleep(0.05)
            continue
        
        # Left side is higher beam indices (counter-clockwise from front)
        # Beams: 0=front, 4,5,6,7 = left side
        front = ranges[0]  # 0 degrees (forward)
        left_front = ranges[5]  # ~90 degrees (left)
        left_rear = ranges[10]  # ~180 degrees (left-rear)
        right_front = ranges[11]  # ~270 degrees (right)
        
        # Invalid reads
        if front < 0:
            front = last_front
        else:
            last_front = front
        
        if left_front < 0:
            left_front = 0.5
        if left_rear < 0:
            left_rear = 0.5
        if right_front < 0:
            right_front = 0.5
        
        left_motor = base_speed
        right_motor = base_speed
        
        # Primary: check for obstacles ahead
        if front < 0.35:
            # Obstacle ahead, must turn right
            left_motor = base_speed
            right_motor = base_speed * -1
        else:
            # Wall following: keep left wall at target distance
            error = left_front - target_distance
            
            if error > tolerance:  # Too far from left wall, turn left
                left_motor = base_speed - 40
                right_motor = base_speed + 40
            elif error < -tolerance:  # Too close to left wall, turn right
                left_motor = base_speed + 40
                right_motor = base_speed - 40
            else:
                # Good distance, go straight
                left_motor = base_speed
                right_motor = base_speed
        
        write_motor(left_motor, right_motor)
        
        iteration += 1
        if iteration % 50 == 0:
            status = read_device(STATUS_DEV)
            print(f"Iter {iteration}: front={front:.2f}m, left={left_front:.2f}m, status={status}")
        
        time.sleep(0.04)
    
    print("Max iterations reached")
    write_motor(0, 0)
    return False

if __name__ == "__main__":
    wall_follow_left()
