import time, sys, math, random
sys.path.insert(0,"/bot/src")
from robot import *
LOG=open("/bot/src/homeA.log","a",buffering=1)
def log(s): LOG.write("%.1f %s\n"%(time.time(),s))
def d11(n=6):
    vals=[]
    for _ in range(n):
        v=rd("d11")
        if v:
            try: vals.append(float(v))
            except: pass
        time.sleep(0.03)
    return sum(vals)/len(vals) if vals else 9
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
log("HOME A START")
t0=time.time()
th_t=None
best=9; last_improve=time.time()
vprev=d11(8)
while time.time()-t0<5000:
    s3=rd("d3",timeout=0.2) or ""
    if "here=1" in s3:
        log("HERE=1 - ON GOAL. Staying.")
        motors(0,0)
        time.sleep(2)
        continue
    v=d11(5)
    l=lid()
    b8=clean(l)[8] if l else 0
    if v<0.30:
        # very close: stop and hold
        log("CLOSE d11=%.3f - holding"%v)
        motors(0,0); time.sleep(1.5)
        continue
    if v < best-0.015:
        best=v; last_improve=time.time()
        log("IMPROVE d11=%.3f - keep heading"%v)
    # navigation decision
    if b8<0.45:
        cands=[(i,(clean(l)[i]**2)) for i in range(16) if clean(l)[i]>0.4 and i!=8]
        if not cands: cands=[(1,1),(15,1)]
        tot=sum(w for _,w in cands); r=random.uniform(0,tot); acc=0; picki=cands[0][0]
        for i,w in cands:
            acc+=w
            if r<=acc: picki=i; break
        rotate_to((havg()+(picki-8)*22.5)%360); th_t=None; continue
    if time.time()-last_improve>25:
        # no improvement for 25s: random detour
        deg=random.choice([-90,-60,60,90])
        log("NORPROG d11=%.3f detour %d"%(v,deg))
        rotate_to((havg()+deg)%360); th_t=None; last_improve=time.time(); continue
    if th_t is None: th_t=havg()
    dl=min(clean(l)[4],clean(l)[5]); dr=min(clean(l)[11],clean(l)[12])
    if dl>1.0 and clean(l)[1]>0.9 and b8>0.6:
        rotate_to((havg()-50)%360); th_t=None; continue
    if dr>1.0 and clean(l)[15]>0.9 and b8>0.6:
        rotate_to((havg()+50)%360); th_t=None; continue
    if dl<0.20: th_t=havg()+7
    elif dr<0.15: th_t=havg()-6
    dh=wrap(th_t-havg())
    base=11
    lb=base+int(round(0.09*dh)); rb=base-int(round(0.09*dh))
    motors(max(4,min(12,lb)),max(4,min(12,rb)))
    time.sleep(0.25)
motors(0,0)
log("HOMEA END")
