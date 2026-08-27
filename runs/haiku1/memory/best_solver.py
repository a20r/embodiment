#!/usr/bin/env python3
"""
Extended maze solver with higher iteration limit.
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

print("Extended solver started", file=sys.stderr, flush=True)

iteration = 0
max_iter = 15000  # Extended limit
base_speed = 165

while iteration < max_iter:
    if check_goal():
        print("GOAL REACHED!", file=sys.stderr, flush=True)
        write_motor(0, 0)
        sys.exit(0)
    
    lidar_str = read_device(LIDAR_DEV)
    if not lidar_str:
        time.sleep(0.01)
        iteration += 1
        continue
    
    ranges = parse_lidar(lidar_str)
    if not ranges or len(ranges) < 16:
        time.sleep(0.01)
        iteration += 1
        continue
    
    for i in range(len(ranges)):
        if ranges[i] < 0:
            ranges[i] = 2.5
    
    f = ranges[0]
    fl = ranges[3]
    fr = ranges[13]
    
    lm = base_speed
    rm = base_speed
    
    if f < 0.32:
        lm = base_speed
        rm = 35
        c = "H"
    elif f < 0.42:
        lm = base_speed
        rm = 75
        c = "R"
    elif f < 0.52:
        lm = base_speed
        rm = 105
        c = "r"
    elif fl < 0.26:
        lm = base_speed + 40
        rm = base_speed - 40
        c = "L"
    elif fr < 0.26:
        lm = base_speed - 40
        rm = base_speed + 40
        c = "R"
    else:
        c = "F"
    
    write_motor(lm, rm)
    
    iteration += 1
    
    if iteration % 300 == 0 or iteration % 1000 < 50:
        st = read_device(STATUS_DEV)
        print(f"I={iteration} F={f:.2f} L={fl:.2f} {c} {st}", file=sys.stderr, flush=True)
        sys.stderr.flush()
    
    time.sleep(0.011)

write_motor(0, 0)
print("Max iterations reached", file=sys.stderr, flush=True)
