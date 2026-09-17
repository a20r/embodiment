import time
while True:
    try:
        with open('/dev/robot/d5') as f:
            for line in f:
                with open('/tmp/d5.log','a') as o:
                    o.write(str(round(time.time(),1))+' '+line)
    except Exception:
        time.sleep(0.5)
