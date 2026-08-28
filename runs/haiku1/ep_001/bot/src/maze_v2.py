#!/usr/bin/env python3
"""
Improved maze solver with better exploration strategy.
"""

import time

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

def write_motor(left, right):
    try:
        with open(MOTOR_LEFT_DEV, 'w') as f:
            f.write(str(int(left)) + '\n')
        with open(MOTOR_RIGHT_DEV, 'w') as f:
            f.write(str(int(right)) + '\n')
    except:
        pass

def parse_lidar(s):
    try:
        return [float(x) for x in s.split(',')]
    except:
        return None

def check_goal():
    s = read_device(STATUS_DEV)
    return s and "goal=1" in s

def solve():
    print("Maze solver v2 starting...", flush=True)
    base_speed = 160
    iteration = 0
    max_it = 6000
    
    while iteration < max_it:
        if check_goal():
            print("GOAL!", flush=True)
            write_motor(0, 0)
            return
        
        lidar_str = read_device(LIDAR_DEV)
        if not lidar_str:
            time.sleep(0.01)
            continue
        
        ranges = parse_lidar(lidar_str)
        if not ranges or len(ranges) < 16:
            time.sleep(0.01)
            continue
        
        # Handle negatives
        for i in range(len(ranges)):
            if ranges[i] < 0:
                ranges[i] = 2.5
        
        # Analyze all angles
        front = ranges[0]
        fwd_left = ranges[2]
        left = ranges[6]
        rear = ranges[12]
        fwd_right = ranges[14]
        
        # Adaptive control based on environment
        lm = base_speed
        rm = base_speed
        
        if front < 0.30:
            # Close wall ahead - turn right
            lm = base_speed
            rm = 30
            cmd = "HARD_RIGHT"
        elif front < 0.40:
            # Wall ahead - turn right
            lm = base_speed
            rm = 60
            cmd = "TURN_RIGHT"
        elif front < 0.50:
            # Approaching wall - light turn
            lm = base_speed
            rm = 100
            cmd = "LIGHT_RIGHT"
        elif fwd_left < 0.25:
            # Wall on left diagonal
            lm = base_speed + 40
            rm = base_speed - 40
            cmd = "WALL_LEFT"
        elif fwd_right < 0.25:
            # Wall on right diagonal
            lm = base_speed - 40
            rm = base_speed + 40
            cmd = "WALL_RIGHT"
        else:
            # Clear path
            cmd = "FORWARD"
        
        write_motor(lm, rm)
        
        iteration += 1
        if iteration % 200 == 0:
            st = read_device(STATUS_DEV)
            print(f"It={iteration}: F={front:.2f} FL={fwd_left:.2f} FR={fwd_right:.2f} {cmd} {st}", flush=True)
        
        time.sleep(0.015)
    
    write_motor(0, 0)

if __name__ == "__main__":
    solve()
