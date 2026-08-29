import time
def rline(p):
    for _ in range(20):
        with open(f'/dev/robot/d{p}') as f:
            s=f.readline().strip()
        if s: return s
        time.sleep(0.02)
    return ''
def w(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
def lidar():
    for _ in range(20):
        s=rline(3)
        try:
            l=[float(x) for x in s.split(',')]
            if len(l)==16: return l
        except: pass
        time.sleep(0.02)
    raise RuntimeError('lidar fail')
def heading():
    return float(rline(1))
def motors(a,b):
    w(10,a); w(11,b)
