import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
LOG=open("/bot/src/exp4.log","a",buffering=1)
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
t0=time.time(); lastlog=0
th_t=None
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
while time.time()-t0<4800:
    if rd("d0")=="1":
        log("BUMP ticks=%d"%ticks()); motors(-5,-5); time.sleep(0.9); motors(0,0); time.sleep(0.4); th_t=None; continue
    l=lid()
    if not l: continue
    b=clean(l)
    b8=b[8]
    if b8<0.45:
        # LEFT-HAND rule: turn CCW ~85 deg
        h=havg(); th_t=(h-85)%360
        log("LEFTTURN h=%.0f b8=%.2f ticks=%d"%(h,b8,ticks()))
        for _ in range(25):
            dh=wrap(th_t-havg())
            if abs(dh)<7: break
            if rd("d0")=="1": motors(-4,-4); time.sleep(0.6); motors(0,0); time.sleep(0.3); continue
            rate=4; t=min(abs(dh)/(2.2*rate),1.2)
            motors(-rate,rate); time.sleep(t); motors(0,0); time.sleep(0.12)
        motors(0,0); time.sleep(0.2)
        continue
    if th_t is None: th_t=havg()
    # left wall follow: left beams 4,5 (h-90, h-67.5); right beams 11,12
    dl=min(b[4],b[5]); dr=min(b[11],b[12])
    if dl<0.22: th_t=havg()+7
    elif dl>0.65 and b[1]>0.8: th_t=havg()-7
    elif dr<0.15: th_t=havg()-6
    dh=wrap(th_t-havg())
    base=9
    k=0.09
    lb=base+int(round(k*dh)); rb=base-int(round(k*dh))
    lb=max(4,min(12,lb)); rb=max(4,min(12,rb))
    motors(lb,rb)
    if time.time()-lastlog>8:
        lastlog=time.time()
        log("RUN h=%.0f b8=%.2f dl=%.2f dr=%.2f ticks=%d d11=%s d3=%s"%(havg(),b8,dl,dr,ticks(),rd("d11"),rd("d3")))
    time.sleep(0.25)
motors(0,0)
log("END")
