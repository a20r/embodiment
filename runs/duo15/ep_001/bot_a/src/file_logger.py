#!/usr/bin/env python3
import subprocess
import time

def cmd(p,v):
    subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)

def read(p):
    r = subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1)
    return r.stdout.strip()

with open('/tmp/robot_log.txt', 'w') as log:
    log.write("ROBOT SEARCH LOG\n")
    log.flush()
    
    cmd(7, 1)
    
    for i in range(300):
        status = read(3)
        dist = read(6)
        heading = read(4)
        msg = read(10)
        
        log.write(f"{i}: here={'1' if 'here=1' in status else '0'} dist={dist} head={heading}")
        if msg: log.write(f" MSG={msg}")
        log.write("\n")
        
        if 'here=1' in status:
            log.write("GOAL FOUND\n")
            log.flush()
            break
        
        if i % 50 == 0:
            log.flush()
        
        time.sleep(0.2)
    
    cmd(7, 0)
    log.write("END\n")

# Print last 20 lines
with open('/tmp/robot_log.txt') as f:
    lines = f.readlines()
    print(''.join(lines[-20:]))
