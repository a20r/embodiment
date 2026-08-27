import time

def readline(port):
    with open(f'/dev/robot/{port}') as f:
        return f.readline().strip()

def lidar():
    for _ in range(50):
        s = readline('d1')
        try:
            v = [float(x) for x in s.split(',')]
            if len(v) == 16: return v
        except: pass
        time.sleep(0.02)
    raise RuntimeError('lidar fail')

def heading():
    for _ in range(50):
        try: return float(readline('d3'))
        except: time.sleep(0.02)
    raise RuntimeError('heading fail')

def enc():
    for _ in range(50):
        try: return int(readline('d2')), int(readline('d8'))
        except: time.sleep(0.02)
    raise RuntimeError('enc fail')

def status():
    return readline('d4')

def goal():
    for _ in range(50):
        s = readline('d4')
        if 'goal=' in s: return int(s.split('goal=')[1])
        time.sleep(0.02)
    return 0

def wheels(l, r):
    with open('/dev/robot/d6','w') as f: f.write(f'{l}\n')
    with open('/dev/robot/d7','w') as f: f.write(f'{r}\n')

def stop(): wheels(0,0)

def avg_lidar(n=8):
    samples=[]
    for _ in range(n):
        samples.append(lidar()); time.sleep(0.05)
    out=[]
    for i in range(16):
        vals=[s[i] for s in samples if s[i]>=0]
        out.append(sum(vals)/len(vals) if vals else 9.9)
    return out

def turn_to(target, speed=40, tol=4):
    t0=time.time()
    while time.time()-t0<20:
        h=heading()
        d=(target-h)%360
        if d>180: d-=360
        if abs(d)<tol:
            stop(); return True
        s=speed if d>0 else -speed
        # heading increases with clockwise (left fwd, right back)
        wheels(s,-s)
        time.sleep(0.05)
    stop(); return False
