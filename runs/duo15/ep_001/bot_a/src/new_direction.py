import subprocess, time
def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()

# Turn sharply and move in a new direction
print("Turning 90 degrees and trying new direction...")
cmd(1, 90)  # Try turning right
time.sleep(0.5)
cmd(7, 1)   # Move forward

found = False
for i in range(50):
    status = read(3)
    if 'here=1' in status:
        print(f"GOAL FOUND at {i}")
        found = True
        break
    if i % 10 == 0:
        dist = read(6)
        heading = read(4)
        print(f"{i}: dist={dist}, heading={heading}")
    time.sleep(0.3)

cmd(7, 0)

if not found:
    # Try another direction
    print("\nTrying heading 180 degrees...")
    cmd(1, 180)
    time.sleep(0.5)
    cmd(7, 1)
    
    for i in range(50):
        status = read(3)
        if 'here=1' in status:
            print(f"GOAL FOUND at {i}")
            found = True
            break
        if i % 10 == 0:
            dist = read(6)
            heading = read(4)
            print(f"{i}: dist={dist}, heading={heading}")
        time.sleep(0.3)
    
    cmd(7, 0)

print("Search complete")
