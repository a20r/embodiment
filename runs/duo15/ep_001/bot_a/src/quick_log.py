import subprocess, time
def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()

with open('/tmp/quick_log.txt', 'w') as log:
    cmd(7, 1)
    for i in range(100):
        if 'here=1' in read(3):
            log.write(f"GOAL at {i}\n"); break
        dist = read(6); head = read(4)
        log.write(f"{i}: {dist},{head}\n")
        time.sleep(0.3)
    cmd(7, 0)

with open('/tmp/quick_log.txt') as f: print(f.read())
