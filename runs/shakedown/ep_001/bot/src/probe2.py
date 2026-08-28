import time
def r(dev):
    for _ in range(5):
        try:
            with open('/dev/robot/'+dev) as f:
                s=f.read().strip()
            if s: return s
        except: pass
        time.sleep(0.02)
    return ''
def w(dev,v):
    with open('/dev/robot/'+dev,'w') as f: f.write(str(v)+'\n')
print('before', r('heading'), r('encoder_left'), r('encoder_right'))
w('motor_left',-100); w('motor_right',100)
time.sleep(0.5)
w('motor_left',0); w('motor_right',0)
time.sleep(0.3)
print('after', r('heading'), r('encoder_left'), r('encoder_right'))
