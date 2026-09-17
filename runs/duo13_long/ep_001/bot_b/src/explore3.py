import time, sys, math, json
sys.path.insert(0,"/bot/src")
from robot import *
LOG=open("/bot/src/exp3.log","a",buffering=1)
def log(s): LOG.write("%.1f %s\n"%(time.time(),s))
def havg(n=5):
    vals=[heading() for _ in range(n)]
    s=sum(math.sin(math.radians(v)) for v in vals)/n
    c=sum(math.cos(math.radians(v)) for v in vals)/n
    return math.degrees(math.atan2(s,c))%360
def lid():
    for _ in range(3):
        l=lidar()
        if l: return l
        time.sleep(0.04)
def clean(l): return [1.6 if (x is None or x<0) else x for x in l]
def wrap(a): return (a+540)%360-180
sign=+1
turnpref=-1
t0=time.time(); lastlog=0
el=er=0.0
def enc():
    global el,er
    a=rd("d9"); b=rd("d6")
    if a: el=float(a)
    if b: er=float(b)
    return el,er
e0=enc()
def ticks():
    e=enc(); return int(((e[0]-e0[0])+(e[1]-e0[1]))/2)
def pick(b):
    best=None;bs=-9
    for i in range(16):
        ang=abs(i-8)*22.5
        sc=b[i]-ang*0.006+(0.15 if (i-8)*turnpref>0 else 0)
        if sc>bs: bs=sc;best=i
    return best
th_t=None
while time.time()-t0<4500:
    if rd("d0")=="1":
        log("BUMP ticks=%d h=%.0f"%(ticks(),havg()))
        motors(-5,-5); time.sleep(0.9); motors(0,0); time.sleep(0.4)
        th_t=None
        continue
    l=lid()
    if not l: continue
    b=clean(l)
    b8=b[8]
    if b8<0.45:
        best=pick(b); err=best-8
        if err==0: err=turnpref*2
        log("TURN best=%d err=%d ticks=%d b=%s"%(best,err,ticks(),[round(x,1) for x in b]))
        deg=err*22.5
        for attempt in range(6):
            t=min(abs(deg)/(2.2*4),2.2)
            if (deg>0)==(sign>0): motors(4,-4)
            else: motors(-4,4)
            time.sleep(t); motors(0,0); time.sleep(0.2)
            if rd("d0")=="1":
                motors(-4,-4); time.sleep(0.7); motors(0,0); time.sleep(0.3); continue
            l2=lid()
            if l2:
                b2=clean(l2); best2=pick(b2); err2=best2-8
                if abs(err2)<abs(err)*0.6 or best2==best: break
                if abs(err2)>abs(err)+0.5:
                    sign*=-1; log("FLIP sign=%d"%sign)
                deg=err2*22.5
        motors(0,0); time.sleep(0.2)
        th_t=None
        continue
    # straight with heading hold
    if th_t is None: th_t=havg()
    dh=wrap(th_t-havg())
    # wall centering adjust target
    if b[10]<0.25 and b[10]<b[6]: th_t=th_t-6   # right wall close: steer left (heading -)
    elif b[6]<0.25 and b[6]<b[10]: th_t=th_t+6  # left wall close: steer right
    th_t=(th_t+ (1.5 if dh==0 else 0))%360       # drift trim
    dh=wrap(th_t-havg())
    base=8
    k=0.10
    lb=base+int(round(k*dh)); rb=base-int(round(k*dh))
    lb=max(3,min(11,lb)); rb=max(3,min(11,rb))
    motors(lb,rb)
    if time.time()-lastlog>8:
        lastlog=time.time()
        log("RUN h=%.0f th_t=%.0f dh=%.0f b8=%.2f b6=%.2f b10=%.2f ticks=%d d11=%s d3=%s"%(havg(),th_t,dh,b8,b[6],b[10],ticks(),rd("d11"),rd("d3")))
    time.sleep(0.25)
motors(0,0)
log("END")
