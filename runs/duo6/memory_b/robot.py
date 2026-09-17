import time

DEV='/dev/robot/'
def rd_raw(p):
    with open(DEV+p) as f: return f.readline().strip()
def rd(p, tries=6):
    for _ in range(tries):
        s=rd_raw(p)
        if s: return s
        time.sleep(0.02)
    return ''
def wr(p,s):
    with open(DEV+p,'w') as f: f.write(str(s)+'\n')
def heading(n=5):
    import math
    xs=ys=0.0; c=0
    for _ in range(n):
        s=rd('d1')
        try: a=math.radians(float(s))
        except: continue
        xs+=math.cos(a); ys+=math.sin(a); c+=1
    import math as m
    if c==0: return None
    return m.degrees(m.atan2(ys,xs))%360
def lidar():
    for _ in range(5):
        s=rd('d3')
        try:
            v=[float(x) for x in s.split(',')]
            if len(v)==16: return v
        except: pass
    return None
def enc():
    for _ in range(10):
        try:
            return int(rd('d7')), int(rd('d8'))
        except ValueError:
            time.sleep(0.02)
    raise RuntimeError('enc fail')
def motors(l,r):
    wr('d10',int(l)); wr('d11',int(r))
def stop(): motors(0,0)
def status():
    s=rd('d6'); d={}
    for kv in s.split():
        if '=' in kv:
            k,v=kv.split('='); d[k]=v
    return d
