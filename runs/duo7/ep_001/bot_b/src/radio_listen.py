import time
while True:
    with open('/dev/robot/d4') as f:
        line=f.readline().strip()
    if line:
        with open('/tmp/radio_rx.log','a') as g:
            g.write('%.1f %s\n'%(time.time(),line))
    else:
        time.sleep(0.2)
