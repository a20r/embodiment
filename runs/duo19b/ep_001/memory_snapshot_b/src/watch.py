import time, os, re
n0 = sum(1 for _ in open('/bot/rx.log'))
while True:
    lines = open('/bot/rx.log').read().splitlines()
    new = lines[n0:]
    for l in new:
        if re.search(r'B ON|here=1|back on|stepped on', l, re.I):
            with open('/bot/explore.log','a') as f: f.write(f"{time.strftime('%H:%M:%S')} WATCH trigger: {l[:80]}\n")
            os.system('cd /bot && python3 src/rearrive.py >> /bot/rearrive.out 2>&1')
            with open('/bot/explore.log','a') as f: f.write(f"{time.strftime('%H:%M:%S')} WATCH rearrive done\n")
            raise SystemExit
    n0 = len(lines)
    time.sleep(1)
