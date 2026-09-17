import time
g=open('/memory/radio_rx.log','a',buffering=1)
while True:
    try:
        with open('/dev/robot/d4') as f:
            got=False
            for line in f:
                s=line.strip()
                got=True
                if s: g.write('%.1f %s\n'%(time.time(),s))
        if not got: time.sleep(0.2)
    except Exception:
        time.sleep(0.5)
