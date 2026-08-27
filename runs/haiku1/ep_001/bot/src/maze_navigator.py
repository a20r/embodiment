#!/usr/bin/env python3

import time
import os

# Device paths
LIDAR_DEV = "/dev/robot/lidar"
STATUS_DEV = "/dev/robot/status"
HEADING_DEV = "/dev/robot/heading"
MOTOR_LEFT_DEV = "/dev/robot/motor_left"
MOTOR_RIGHT_DEV = "/dev/robot/motor_right"

def read_device(device_path):
    """Read a device file and return the value"""
    try:
        with open(device_path, 'r') as f:
            return f.read().strip()
    except Exception as e:
        print(f"Error reading {device_path}: {e}")
        return None

def write_motor(left_speed, right_speed):
    """Write motor commands"""
    try:
        with open(MOTOR_LEFT_DEV, 'w') as f:
            f.write(str(left_speed) + '\n')
        with open(MOTOR_RIGHT_DEV, 'w') as f:
            f.write(str(right_speed) + '\n')
    except Exception as e:
        print(f"Error writing motors: {e}")

def parse_lidar(lidar_str):
    """Parse lidar string into list of ranges"""
    try:
        ranges = [float(x) for x in lidar_str.split(',')]
        return ranges
    except Exception as e:
        print(f"Error parsing lidar: {e}")
        return None

def check_goal():
    """Check if we've reached the goal"""
    status = read_device(STATUS_DEV)
    if status and "goal=1" in status:
        return True
    return False

def navigate():
    """Main navigation loop using wall-following or obstacle avoidance"""
    print("Starting maze navigation...")
    
    max_iterations = 1000
    iteration = 0
    
    while iteration < max_iterations:
        # Check if goal reached
        if check_goal():
            print("GOAL REACHED!")
            write_motor(0, 0)  # Stop
            return True
        
        # Read sensors
        lidar_str = read_device(LIDAR_DEV)
        if not lidar_str:
            time.sleep(0.1)
            continue
        
        ranges = parse_lidar(lidar_str)
        if not ranges or len(ranges) < 16:
            time.sleep(0.1)
            continue
        
        # Simple obstacle avoidance: detect obstacles in front and turn away
        # Beam 0 is forward, beams increase counter-clockwise
        front_distance = ranges[0]
        front_left_distance = ranges[2]  # ~45 degrees left
        front_right_distance = ranges[14]  # ~45 degrees right
        
        # Basic strategy: if something in front, turn
        base_speed = 100
        left_motor = base_speed
        right_motor = base_speed
        
        # If obstacle ahead, turn right
        if front_distance < 0.5:
            left_motor = base_speed
            right_motor = -base_speed
            print(f"Obstacle ahead ({front_distance:.2f}m), turning right")
        # If wall on right, turn left slightly
        elif front_right_distance < 0.3:
            left_motor = base_speed - 30
            right_motor = base_speed + 30
            print(f"Wall on right ({front_right_distance:.2f}m), adjusting left")
        # If wall on left, turn right slightly
        elif front_left_distance < 0.3:
            left_motor = base_speed + 30
            right_motor = base_speed - 30
            print(f"Wall on left ({front_left_distance:.2f}m), adjusting right")
        
        write_motor(left_motor, right_motor)
        
        iteration += 1
        if iteration % 20 == 0:
            print(f"Iteration {iteration}: front={front_distance:.2f}m, status={read_device(STATUS_DEV)}")
        
        time.sleep(0.05)
    
    print("Max iterations reached")
    write_motor(0, 0)
    return False

if __name__ == "__main__":
    navigate()
