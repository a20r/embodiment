import time
def w(p,v):
    with open(f'/dev/robot/{p}','w') as f: f.write(str(v)+'\n')
def r(p):
    for _ in range(20):
        with open(f'/dev/robot/{p}') as f:
            s=f.readline().strip()
        if s: return s
        time.sleep(0.05)
    return ''
def rf(p): return float(r(p))
def ri(p): return int(r(p))
def lidar(): return [float(x) for x in r('d2').split(',')]
def stop(): w('d1',0); w('d7',0)

def heading(): return rf('d4')
def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d
def turn_to(target, tol=4):
    for _ in range(120):
        h=heading()
        e=angdiff(target,h)
        if abs(e)<=tol:
            stop(); return True
        s=max(3,min(15,abs(e)*0.4))
        if e>0: w('d1',s); w('d7',-s)
        else: w('d1',-s); w('d7',s)
        time.sleep(0.12)
    stop(); return False
def turn_by(d, tol=4):
    return turn_to(heading()+d, tol)
def bump(): return r('d5')=='1'
