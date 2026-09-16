import time, math, signal, sys
from robot import Robot
r = Robot()
def stopall(*a):
    r.stop(); sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)
r.stop(); time.sleep(0.3)
TICKS=544.0
def beam(lid,i):
    v = lid[i%16]
    if v is None or v<0:
        a=lid[(i-1)%16]; b=lid[(i+1)%16]
        c=[z for z in (a,b) if z and z>0]
        v=min(c) if c else 0.5
    return v
def front(lid): return min(beam(lid,15),beam(lid,0),beam(lid,1))
def rotto(tg,timeout=6):
    t0=time.time()
    while time.time()-t0<timeout:
        hc=r.heading()
        if hc is None: continue
        e=(tg-hc+180)%360-180
        if abs(e)<=4: break
        v=max(10,min(25,abs(e)))
        if e>0: r.wheels(v,-v)
        else: r.wheels(-v,v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.1)
best=None
for i in range(8):
    lid=r.lidar(); h=r.heading()
    f=front(lid)
    if best is None or f>best[0]: best=(f,h)
    rotto((h+45)%360)
rotto(best[1])
print('best dir h=%.0f front=%.2f' % best, flush=True)
x=0.0
for i in range(40):
    lid=r.lidar(); f=front(lid)
    print(f'dist={x:.2f} front={f:.2f} lid={[round(v,2) for v in lid]}', flush=True)
    if f<0.18:
        print('blocked', flush=True); break
    l0,r0=r.enc()
    t0=time.time()
    while time.time()-t0<4:
        l1,r1=r.enc()
        if l1 is None: continue
        prog=((l1-l0)+(r1-r0))/2
        if prog>=0.04*TICKS: break
        if front(r.lidar() or [])<0.16: break
        r.wheels(12,12); time.sleep(0.03)
    r.stop(); time.sleep(0.12)
    x+=0.04
r.stop()
print('done x=%.2f'%x, flush=True)
