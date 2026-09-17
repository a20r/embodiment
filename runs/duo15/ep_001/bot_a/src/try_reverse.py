import subprocess, time
def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()

with open('/tmp/reverse_log.txt', 'w') as log:
    # Try going backwards
    print("Trying reverse direction...")
    cmd(7, -1)
    for i in range(100):
        status = read(3)
        dist = read(6)
        if 'here=1' in status:
            print(f"GOAL at {i}")
            log.write(f"GOAL at {i}\n")
            break
        if i % 10 == 0: print(f"{i}: {dist}")
        log.write(f"{i}: {status.split()[2]}\n")
        time.sleep(0.3)
    cmd(7, 0)
    print("Done")

with open('/tmp/reverse_log.txt') as f: print(f.read()[-500:])
