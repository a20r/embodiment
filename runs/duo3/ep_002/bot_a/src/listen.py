import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port
with open('/tmp/radio.log','a') as f:
    while True:
        s = read_port('d3', timeout=5)
        if s and s.strip():
            f.write(f"{time.time():.1f} {s}\n"); f.flush()
        elif s is None:
            time.sleep(0.02)
