#!/usr/bin/env python3
"""
Maze solver for MZB-1 robot using right-hand wall following
with adaptive speed control.
"""

import time
import sys

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

def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))

def solve_maze():
    """
    Main maze solving function.
    Uses a combination of forward driving and wall-following.
    """
    print("Starting maze solver...", flush=True)
    
    base_speed = 150
    iteration = 0
    max_iterations = 5000
    stuck_counter = 0
    last_front_distances = []
    
    while iteration < max_iterations:
        # Check goal
        if check_goal():
            print("GOAL REACHED!", flush=True)
            write_motor(0, 0)
            time.sleep(0.5)
            return True
        
        # Read lidar
        lidar_str = read_device(LIDAR_DEV)
        if not lidar_str:
            time.sleep(0.02)
            continue
        
        ranges = parse_lidar(lidar_str)
        if not ranges or len(ranges) < 16:
            time.sleep(0.02)
            continue
        
        # Handle invalid readings
        for i in range(len(ranges)):
            if ranges[i] < 0:
                ranges[i] = 2.0
        
        # Key measurements (using beam indices 0-15)
        front = ranges[0]          # 0 degrees
        front_left_45 = ranges[3]  # 45 degrees left
        left = ranges[6]           # 90 degrees left
        rear_left = ranges[9]      # 135 degrees (rear-left)
        rear = ranges[12]          # 180 degrees rear
        rear_right = ranges[15]    # 225 degrees (rear-right)
        front_right_45 = ranges[13]  # 315 degrees (45 degrees right-forward)
        right = ranges[10]         # 270 degrees right
        
        # Track if stuck
        last_front_distances.append(front)
        if len(last_front_distances) > 30:
            last_front_distances.pop(0)
            avg_front = sum(last_front_distances) / len(last_front_distances)
            if avg_front < 0.25:
                stuck_counter += 1
            else:
                stuck_counter = 0
        
        # If stuck, try backing up and turning
        if stuck_counter > 20:
            print(f"Stuck! Backing up... (iter {iteration})", flush=True)
            write_motor(-100, -100)
            time.sleep(0.3)
            write_motor(-80, 80)
            time.sleep(0.3)
            stuck_counter = 0
            last_front_distances = []
        
        # Main control logic
        left_motor = base_speed
        right_motor = base_speed
        
        # If significantly blocked ahead, turn sharply right
        if front < 0.35:
            # Hard obstacle ahead
            left_motor = base_speed
            right_motor = base_speed * 0.2
            action = "TURN_RIGHT"
        elif front < 0.45:
            # Moderate obstacle
            left_motor = base_speed
            right_motor = base_speed * 0.5
            action = "TURN_RIGHT_MED"
        elif front_left_45 < 0.30:
            # Wall diagonal left, need to turn right
            left_motor = base_speed + 30
            right_motor = base_speed - 30
            action = "AVOID_LEFT"
        elif front_right_45 < 0.25:
            # Wall on right, turn left
            left_motor = base_speed - 40
            right_motor = base_speed + 40
            action = "AVOID_RIGHT"
        elif left < 0.30:
            # Wall on left, turn right
            left_motor = base_speed + 20
            right_motor = base_speed - 20
            action = "LEFT_WALL"
        elif right < 0.25:
            # Wall on right, turn left
            left_motor = base_speed - 20
            right_motor = base_speed + 20
            action = "RIGHT_WALL"
        else:
            # Open path
            left_motor = base_speed
            right_motor = base_speed
            action = "FORWARD"
        
        # Limit motor speeds
        left_motor = clamp(left_motor, -255, 255)
        right_motor = clamp(right_motor, -255, 255)
        
        write_motor(left_motor, right_motor)
        
        iteration += 1
        if iteration % 150 == 0:
            status_str = read_device(STATUS_DEV)
            print(f"It={iteration}: F={front:.2f} FL={front_left_45:.2f} FR={front_right_45:.2f} | {action} | {status_str}", flush=True)
        
        time.sleep(0.02)
    
    print("Max iterations reached", flush=True)
    write_motor(0, 0)
    return False

if __name__ == "__main__":
    try:
        solve_maze()
    except Exception as e:
        print(f"Error: {e}", flush=True)
        write_motor(0, 0)
