import time

D = '/dev/robot/'

def readline(port, timeout=2.0):
    t0 = time.time()
    while time.time()-t0 < timeout:
        with open(D+port) as f:
            s = f.readline().strip()
        if s:
            return s
        time.sleep(0.05)
    return None

def write(port, s):
    with open(D+port, 'w') as f:
        f.write(str(s)+'\n')

def heading():
    return float(readline('d1'))

def lidar():
    return [float(x) for x in readline('d3').split(',')]

def enc():
    return int(readline('d7')), int(readline('d8'))

def motors(l, r):
    write('d10', l); write('d11', r)

def status():
    return readline('d6')

def tx(s): write('d0', s)
def rx(): return readline('d4')

def angdiff(a, b):
    """smallest signed diff a-b in degrees, range [-180,180)"""
    return (a - b + 180) % 360 - 180

def turn_to(target, speed=10, tol=3):
    while True:
        h = heading()
        d = angdiff(target, h)
        if abs(d) <= tol:
            motors(0,0); return h
        s = speed if d > 0 else -speed
        # slow near target
        if abs(d) < 15: s = s//3 or (1 if d>0 else -1)
        motors(s, -s)
        time.sleep(0.08)

def avg_lidar(n=4):
    samples=[lidar() for _ in range(n)]
    out=[]
    for i in range(16):
        vals=[s[i] for s in samples if s[i]>0]
        out.append(sum(vals)/len(vals) if vals else -1)
    return out
