import time
while True:
    try:
        with open('/dev/robot/d4') as f:
            for line in f:
                with open('/tmp/rx.log','a') as o:
                    o.write(str(round(time.time(),1))+" "+line) if line.strip() else None
    except Exception:
        time.sleep(0.5)
