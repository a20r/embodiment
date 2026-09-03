import time
def rd(p):
    with open('/dev/robot/'+p) as f: return f.readline().strip()
while True:
    try:
        s=rd('d6'); d2=rd('d2')
        with open('/tmp/mon.log','a') as f:
            f.write('%.0f %s d2=%s\n'%(time.time(),s,d2))
    except Exception as e:
        pass
    time.sleep(5)
