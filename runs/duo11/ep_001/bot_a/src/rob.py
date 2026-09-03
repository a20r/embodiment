import time

DEV='/dev/robot/'
def rd(port):
    for _ in range(50):
        with open(DEV+port) as f:
            s=f.readline().strip()
        if s: return s
        time.sleep(0.05)
    return s
def wr(port,val):
    with open(DEV+port,'w') as f:
        f.write(str(val)+'\n')
def scan():
    s=rd('d2')
    return [float(x) for x in s.split(',')]
def heading():
    return float(rd('d4'))
def status():
    return rd('d3')
def enc():
    return int(rd('d6')), int(rd('d9'))
def drive(v): wr('d1',v)
def turn(w): wr('d7',w)
def tx(msg): wr('d8',msg)

def norm(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a

def turn_to(target, tol=4):
    for _ in range(200):
        h=heading()
        err=norm(target-h)
        if abs(err)<tol:
            turn(0); return h
        # positive turn decreases heading
        rate=max(min(-err*0.8, 30), -30)
        if 0<rate<3: rate=3
        if -3<rate<0: rate=-3
        turn(rate)
        time.sleep(0.2)
    turn(0)
    return heading()
