import sys; sys.path.insert(0,'/bot/src')
from lib import *
import os, time, select

def grab_scan(dur=0.5):
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
    for ln in buf.split('\n'):
        for p in ln.split(';'):
            if p and ',' in p:
                try:
                    a,b,c=map(float,p.split(','))
                    if -0.2<=c<=0.25 and b>=-0.05: pts.append((a,c))
                except ValueError: pass
    return pts

def front_min():
    pts=grab_scan(0.35)
    if not pts: return None
    # center columns only
    c=[a for a,az in pts if abs(az)<0.08]
    return min(c) if c else None

stop(); time.sleep(0.3)
print("front min range:", front_min(), flush=True)
e0=int(last_of('d6')); e9_0=int(last_of('d9'))
r0=front_min()
write('d1',30); write('d7',30)
t0=time.time()
while time.time()-t0<3.5:
    m=front_min()
    if m and m<0.24: break
stop()
e1=int(last_of('d6')); e9_1=int(last_of('d9'))
r1=front_min()
dt=time.time()-t0
print(f"r0={r0} r1={r1} ticks L={e1-e0} R={e9_1-e9_0} dt={dt:.1f}", flush=True)
if r0 and r1 and r0>r1:
    dist=r0-r1; ticks=((e1-e0)+(e9_1-e9_0))/2
    print(f"closed {dist:.3f} m with {ticks} ticks => {ticks/dist:.1f} ticks/m")
