import time
while True:
    try:
        with open('/dev/robot/d4') as f:
            for line in f:
                if line.strip():
                    with open('/tmp/rx.log','a') as o:
                        o.write(str(round(time.time(),1))+' '+line)
    except Exception:
        time.sleep(0.5)
