import time, sys, math, json
sys.path.insert(0,"/bot/src")
from robot import *
LOG=open("/bot/src/exp2.log","a",buffering=1)
def log(s): LOG.write("%.1f %s\n"%(time.time(),s))
def havg(n=4):
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
sign=+1
turnpref=-1  # left-hand
t0=time.time()
lastlog=0
el=er=0
def enc():
    global el,er
    a=rd("d9"); b=rd("d6")
    if a: el=float(a)
    if b: er=float(b)
    return el,er
e0=enc()
def ticks(): 
    e=enc(); return int(((e[0]-e0[0])+(e[1]-e0[1]))/2)
while time.time()-t0<4500:
    if rd("d0")=="1":
        log("BUMP ticks=%d h=%.0f"%(ticks(),havg()))
        motors(-5,-5); time.sleep(0.9); motors(0,0); time.sleep(0.4)
        continue
    l=lid()
    if not l: continue
    b=clean(l)
    b8=b[8]
    if b8<0.42:
        # pick direction: score all beams
        best=None;bs=-9
        for i in range(16):
            ang=abs(i-8)*22.5
            sc=b[i]-ang*0.006+(0.15 if (i-8)*turnpref>0 else 0)
            if sc>bs: bs=sc;best=i
        err=best-8
        if err==0: err=turnpref*2
        log("TURN best=%d err=%d b=%s ticks=%d"%(best,err,[round(x,2) for x in b],ticks()))
        deg=err*22.5
        for attempt in range(6):
            t=abs(deg)/(2.2*4)
            if (deg>0)==(sign>0): motors(4,-4)
            else: motors(-4,4)
            time.sleep(min(t,2.2)); motors(0,0); time.sleep(0.2)
            if rd("d0")=="1":
                motors(-4,-4); time.sleep(0.7); motors(0,0); time.sleep(0.3)
                continue
            l2=lid()
            if l2:
                b2=clean(l2); best2=None;bs2=-9
                for i in range(16):
                    ang=abs(i-8)*22.5
                    sc=b2[i]-ang*0.006+(0.15 if (i-8)*turnpref>0 else 0)
                    if sc>bs2: bs2=sc;best2=i
                err2=best2-8
                if abs(err2)<abs(err)*0.6 or (best2==best): break
                if abs(err2)>abs(err)+0.5:
                    sign*=-1
                    log("FLIP sign=%d"%sign)
                deg=err2*22.5
        motors(0,0); time.sleep(0.2)
        continue
    # drive with centering
    lft=(b[5]+b[6])/2; rgt=(b[10]+b[11])/2
    lb=rb=8
    d=lft-rgt
    if d<-0.15: lb=7; rb=8
    elif d>0.15: lb=8; rb=7
    motors(lb,rb)
    if time.time()-lastlog>8:
        lastlog=time.time()
        log("RUN h=%.0f b8=%.2f b56=%.2f b1011=%.2f ticks=%d d11=%s d3=%s d5=%s"%(havg(),b8,lft,rgt,ticks(),rd("d11"),rd("d3"),rd("d5")))
    time.sleep(0.25)
motors(0,0)
log("END")
