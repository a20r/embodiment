import subprocess, time, threading

def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()

found_msg = False

def broadcaster():
    for i in range(100):
        cmd(8, f"ROBOT_1_SEARCHING_{i}")
        time.sleep(0.5)

def listener():
    global found_msg
    for i in range(200):
        msg = read(10)
        if msg and len(msg) > 2:
            print(f"RX: {msg}")
            found_msg = True
            return
        time.sleep(0.25)

# Start broadcast and listen threads
threading.Thread(target=broadcaster, daemon=True).start()
threading.Thread(target=listener, daemon=True).start()

# Also move
cmd(7, 1)
for i in range(100):
    status = read(3)
    if found_msg or 'here=1' in status:
        print(f"FOUND at {i}")
        break
    if i % 20 == 0:
        dist = read(6)
        print(f"{i}: dist={dist}")
    time.sleep(0.3)

cmd(7, 0)
print("Done")
