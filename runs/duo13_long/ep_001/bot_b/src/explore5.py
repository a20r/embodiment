import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
LOG=open("/bot/src/exp5.log","a",buffering=1)
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
t0=time.time(); lastlog=0; th_t=None
consec_turns=0; last_turn_tick=0
turnpref=+1
sigs=[]
orbit_acc=0.0; orbit_mode=0; orbit_t=0; last_turn_wall=time.time()
px=py=pth=None
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
        sc=b[i]-ang*0.008+(0.25 if (i-8)*turnpref>0 else 0)
        if sc>bs: bs=sc;best=i
    return best
def note_sig(b):
    s=tuple(int(round(x*2)) for x in b)
    sigs.append(s)
    if len(sigs)>14: sigs.pop(0)
    return sigs.count(s)
def rotate_by(deg):
    # deg>0 CW (heading +)
    for _ in range(30):
        if rd("d0")=="1":
            motors(-5,-5); time.sleep(0.8); motors(0,0); time.sleep(0.4); continue
        cur=havg()
        remaining=deg
        if abs(remaining)<8: return True
        rate=4; t=min(abs(remaining)/(2.2*rate),1.0)
        if remaining>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.12)
        deg-= (havg()-cur) if False else 0
        # recompute remaining from target directly:
    return False
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
while time.time()-t0<4800:
    if rd("d0")=="1":
        log("BUMP ticks=%d"%ticks()); motors(-5,-5); time.sleep(0.9); motors(0,0); time.sleep(0.4); th_t=None; continue
    l=lid()
    if not l: continue
    b=clean(l)
    b8=b[8]
    if b8<0.45:
        tk=ticks()
        if tk-last_turn_tick<60: consec_turns+=1
        else: consec_turns=0
        last_turn_tick=tk
        if consec_turns>=3:
            log("STUCK: reverse then re-pick ticks=%d b=%s"%(tk,[round(x,1) for x in b]))
            motors(-7,-7); time.sleep(1.6); motors(0,0); time.sleep(0.4)
            consec_turns=0
            l=lid()
            if l: b=clean(l)
        cnt=note_sig(b)
        if cnt>=3:
            turnpref*=-1; sigs.clear()
            log("LOOP! switch side -> %d"%turnpref)
        best=pick(b); err=best-8
        if err==0: err=turnpref*2
        log("TURN best=%d err=%d pref=%d ticks=%d"%(best,err,turnpref,tk))
        orbit_acc=0; last_turn_wall=time.time()
        target=(havg()+err*22.5)%360
        ok=rotate_to(target)
        th_t=None
        time.sleep(0.2)
        continue
    if th_t is None: th_t=havg()
    # orbit detection: integrate heading change while driving
    h_now=havg()
    if pth is not None:
        orbit_acc+=abs(wrap(h_now-pth))
    pth=h_now
    if (orbit_acc>250 or time.time()-last_turn_wall>70) and orbit_mode==0:
        orbit_mode=1; orbit_t=time.time(); orbit_acc=0
        log("ORBIT detected - cutting across")
    if orbit_mode==1:
        th_t=(havg()+40*turnpref)%360
        dh2=wrap(th_t-havg())
        lb=9+int(round(0.09*dh2)); rb=9-int(round(0.09*dh2))
        motors(max(4,min(12,lb)),max(4,min(12,rb)))
        if time.time()-orbit_t>3.5:
            orbit_mode=0; th_t=havg()
        time.sleep(0.25)
        continue
    dl=min(b[4],b[5]); dr=min(b[11],b[12])
    if dl>0.95 and b[1]>0.9 and b8>0.6:
        # wide open left: cross over decisively
        log("CROSS left dl=%.2f h=%.0f"%(dl,havg()))
        rotate_to((havg()-50)%360); th_t=None; time.sleep(0.3)
        continue
    if dl<0.20: th_t=havg()+7
    elif dr<0.15: th_t=havg()-6
    dh=wrap(th_t-havg())
    base=9; k=0.09
    lb=base+int(round(k*dh)); rb=base-int(round(k*dh))
    lb=max(4,min(12,lb)); rb=max(4,min(12,rb))
    motors(lb,rb)
    if time.time()-lastlog>8:
        lastlog=time.time()
        log("RUN h=%.0f b8=%.2f dl=%.2f dr=%.2f ticks=%d d11=%s d3=%s"%(havg(),b8,dl,dr,ticks(),rd("d11"),rd("d3")))
    time.sleep(0.25)
motors(0,0)
log("END")
