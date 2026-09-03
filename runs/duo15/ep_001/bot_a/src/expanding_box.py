import subprocess, time
def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()

print("Expanding box search pattern...")

# Box perimeter = forward, turn right, forward, turn right, etc
# Each iteration of the loop makes a bigger box

for box_size in range(1, 6):
    print(f"\nBox size {box_size}")
    
    for side in range(4):
        # Set heading based on side
        headings = [0, 90, 180, 270]
        cmd(1, headings[side])
        cmd(7, 1)
        
        # Move for a distance proportional to box size
        steps = 10 * box_size
        
        for step in range(steps):
            status = read(3)
            msg = read(10)
            
            if 'here=1' in status:
                print(f"*** GOAL FOUND at box {box_size}, side {side}, step {step}***")
                cmd(7, 0)
                exit(0)
            
            if msg and len(msg) > 0:
                print(f"*** MESSAGE: {msg} ***")
            
            time.sleep(0.2)
        
        cmd(7, 0)

print("Box search complete")
