import sys; sys.path.insert(0,'/bot/src')
from lib import *
import os, time, select, re

def cap(dur=1.2):
    buf=''
    fd=os.open('/dev/robot/d2', os.O_RDONLY|os.O_NONBLOCK)
    t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select([fd],[],[],0.02)
        if r:
            try: buf+=os.read(fd,4096).decode()
            except BlockingIOError: pass
    os.close(fd)
    pts=[]
    for x in buf.replace('\n',';').split(';'):
        m=re.findall(r'-?\d*\.?\d+',x)
        if len(m)==3:
            try: pts.append(tuple(map(float,m)))
            except: pass
    pos=[p for p in pts if p[0]>=0]; neg=[p for p in pts if p[0]<0]
    def st(g):
        if not g: return "empty"
        return f"n={len(g)} r[|c0|] {min(abs(x[0]) for x in g):.2f}-{max(abs(x[0]) for x in g):.2f} mean {sum(abs(x[0]) for x in g)/len(g):.2f} c1 {min(x[1] for x in g):.2f}..{max(x[1] for x in g):.2f} c2 {min(x[2] for x in g):.2f}..{max(x[2] for x in g):.2f}"
    return st(pos), st(neg)

def rot(deg, cmd=60):
    h0=float(last_of('d4'))
    sign=1 if deg>0 else -1
    write('d1',-sign*cmd); write('d7',sign*cmd)
    t0=time.time()
    while time.time()-t0 < abs(deg)/1.74*2*(50/cmd):
        pass
    write('d1',0); write('d7',0)
    time.sleep(0.3)
    h1=float(last_of('d4'))
    return d4norm(h0,h1)

stop(); time.sleep(0.5)
for i in range(4):
    p,n=cap()
    print(f"h={last_of('d4')} POS {p}")
    print(f"        NEG {n}", flush=True)
    rot(90)
print("done")
