import subprocess, time
def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()
def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)

cmd(7, 0)  # Stop moving

print("Waiting in place for 30 seconds...")
for i in range(30):
    status = read(3)
    msg = read(10)
    
    if 'here=1' in status or msg:
        print(f"[{i}] {status}")
        if msg: print(f"    MSG: {msg}")
    
    if i % 5 == 0:
        print(f"[{i}] Waiting...")
    
    time.sleep(1)

print("Done waiting")
