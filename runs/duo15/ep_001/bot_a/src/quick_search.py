import subprocess, time

def cmd(p, v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
def read(p): return subprocess.run(f'timeout 0.3 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True).stdout.strip()

cmd(7, 1)
for i in range(20):
    if i % 5 == 0: cmd(1, i)
    if 'here=1' in read(3) or 'HELLO' in read(10): print(f"FOUND at {i}"); break
    if i % 5 == 0: print(f"{i}: {read(4)}")
    time.sleep(0.5)
cmd(7, 0)
