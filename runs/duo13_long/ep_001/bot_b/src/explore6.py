import time, sys, math, random
sys.path.insert(0,"/bot/src")
from robot import *
LOG=open("/bot/src/exp6.log","a",buffering=1)
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
def rotate_to(target):
    for _ in range(30):
        if rd("d0")=="1":
            motors(-5,-5); time.sleep(0.8); motors(0,0); time.sleep(0.4); continue
        dh=wrap(target-havg())
        if abs(dh)<8: return True
        rate=4; t=min(abs(dh)/(2.2*rate),1.2)
        if dh>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.12)
    return False
K=0.0004
x=y=0.0; th=havg(); lastpos=time.time()
el=er=0.0; ppl=ppr=None
def enc():
    global el,er
    a=rd("d9"); b=rd("d6")
    if a: el=float(a)
    if b: er=float(b)
    return el,er
e0=enc()
def ticks():
    e=enc(); return int(((e[0]-e0[0])+(e[1]-e0[1]))/2)
t0=time.time(); lastlog=0; th_t=None; consec=0; last_turn_tick=0
while time.time()-t0<5400:
    if rd("d0")=="1":
        log("BUMP ticks=%d"%ticks()); motors(-5,-5); time.sleep(0.9); motors(0,0); time.sleep(0.4); th_t=None; continue
    # odometry
    e=enc()
    if ppl is not None:
        dlt=(el-ppl); drt=(er-ppr)
        ds=((dlt+drt)/2)*K
        th=(th+(drt-dlt)*0.174)%360
        x+=ds*math.cos(math.radians(th)); y+=ds*math.sin(math.radians(th))
    ppl,ppr=el,er
    if time.time()-lastpos>30:
        lastpos=time.time()
        log("POSE x=%.2f y=%.2f th=%.0f havg=%.0f ticks=%d d11=%s d3=%s"%(x,y,th,havg(),ticks(),rd("d11"),rd("d3")))
    l=lid()
    if not l: continue
    b=clean(l); b8=b[8]
    if b8<0.32:
        # dead-end niche probe: if this is the most open direction and >0.14, enter slowly first
        if b8>0.14 and max(b)<0.9:
            log("NICHE PROBE b8=%.2f"%b8)
            motors(8,8); t_n=time.time(); bumped=False
            while time.time()-t_n<3.5:
                time.sleep(0.2)
                if rd("d0")=="1": bumped=True; break
                l2=lid()
                if l2 and l2[8]>0 and l2[8]<0.14: break
            motors(-6,-6); time.sleep(1.0); motors(0,0); time.sleep(0.3)
            th_t=None
            if bumped: log("NICHE BUMP")
            l=lid()
            if not l: continue
            b=clean(l); b8=b[8]
            if b8<0.32 and max(b)<0.55:
                pass  # fall through to turn logic
            else:
                continue
        tk=ticks()
        if tk-last_turn_tick<60: consec+=1
        else: consec=0
        last_turn_tick=tk
        if consec>=3:
            log("STUCK reverse"); motors(-7,-7); time.sleep(1.6); motors(0,0); time.sleep(0.4); consec=0
            l=lid(); b=clean(l) if l else b
        # random free direction, weighted by openness^2
        cands=[(i,(b[i]**2)) for i in range(16) if b[i]>0.3 and i!=8]
        if not cands: cands=[(1,1),(15,1)]
        if random.random()<0.25:
            picki=min(cands,key=lambda t:t[1])[0]
            log("POCKET probe beam=%d b=%.2f"%(picki,b[picki]))
        else:
            tot=sum(w for _,w in cands); r=random.uniform(0,tot); acc=0; picki=cands[0][0]
            for i,w in cands:
                acc+=w
                if r<=acc: picki=i; break
        err=picki-8
        if err==0: err=-2
        log("TURN rand best=%d err=%d ticks=%d b8=%.2f"%(picki,err,tk,b8))
        rotate_to((havg()+err*22.5)%360); th_t=None; time.sleep(0.2)
        continue
    if th_t is None: th_t=havg()
    # occasional random detour at open space
    if random.random()<0.008:
        deg=random.choice([-90,-45,45,90])
        l2=lid()
        ok=False
        if l2:
            bb=clean(l2)
            idx=int(round(((deg%360)/22.5)))%16
            i2=(8+ (idx if deg>0 else -idx))%16
            ok=bb[i2]>0.5
        if ok:
            log("DETOUR %d"%deg)
            rotate_to((havg()+deg)%360); th_t=None; continue
    dl=min(b[4],b[5]); dr=min(b[11],b[12])
    if dl>0.85 and b[1]>0.8 and b8>0.5:
        log("CROSS L"); rotate_to((havg()-50)%360); th_t=None; time.sleep(0.3); continue
    if dr>0.85 and b[15]>0.8 and b8>0.5:
        log("CROSS R"); rotate_to((havg()+50)%360); th_t=None; time.sleep(0.3); continue
    if dl<0.20: th_t=havg()+7
    elif dr<0.15: th_t=havg()-6
    dh=wrap(th_t-havg())
    base=15; k=0.09
    lb=base+int(round(k*dh)); rb=base-int(round(k*dh))
    motors(max(4,min(14,lb)),max(4,min(14,rb)))
    if time.time()-lastlog>10:
        lastlog=time.time()
        log("RUN h=%.0f b8=%.2f dl=%.2f dr=%.2f ticks=%d b=%s"%(havg(),b8,dl,dr,ticks(),",".join("%.1f"%v for v in b)))
    time.sleep(0.25)
motors(0,0)
log("END")
