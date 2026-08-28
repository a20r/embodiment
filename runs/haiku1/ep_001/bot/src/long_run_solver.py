#!/usr/bin/env python3
"""
Long-running maze solver with continuous operation.
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
    except Exception as e:
        pass

def parse_lidar(s):
    try:
        return [float(x) for x in s.split(',')]
    except:
        return None

def check_goal():
    s = read_device(STATUS_DEV)
    return s and "goal=1" in s

# Main loop
print("Long-running maze solver started", file=sys.stderr, flush=True)
sys.stderr.flush()

iteration = 0
max_iter = 8000
base_speed = 160

while iteration < max_iter:
    # Check for goal
    if check_goal():
        print("GOAL REACHED!", file=sys.stderr, flush=True)
        write_motor(0, 0)
        sys.exit(0)
    
    # Read sensors
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
    
    # Normalize invalid readings
    for i in range(len(ranges)):
        if ranges[i] < 0:
            ranges[i] = 2.5
    
    # Extract key measurements
    f = ranges[0]      # forward
    fl = ranges[3]     # forward-left 45°
    l = ranges[6]      # left 90°
    bl = ranges[9]     # back-left
    b = ranges[12]     # back
    br = ranges[15]    # back-right
    fr = ranges[13]    # forward-right 45°
    r = ranges[10]     # right 90°
    
    # Navigation logic
    lm = base_speed
    rm = base_speed
    
    if f < 0.32:
        lm = base_speed
        rm = 40
        c = "HR"
    elif f < 0.42:
        lm = base_speed
        rm = 80
        c = "TR"
    elif f < 0.52:
        lm = base_speed
        rm = 110
        c = "LR"
    elif fl < 0.28:
        lm = base_speed + 35
        rm = base_speed - 35
        c = "WL"
    elif fr < 0.28:
        lm = base_speed - 35
        rm = base_speed + 35
        c = "WR"
    else:
        c = "FW"
    
    write_motor(lm, rm)
    
    iteration += 1
    
    if iteration % 250 == 0:
        st = read_device(STATUS_DEV)
        print(f"I={iteration} F={f:.2f} FL={fl:.2f} FR={fr:.2f} {c} {st}", file=sys.stderr, flush=True)
        sys.stderr.flush()
    
    time.sleep(0.012)

write_motor(0, 0)
print("Max iterations reached", file=sys.stderr, flush=True)
