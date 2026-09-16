import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
LOG=open("/bot/src/meet.log","a",buffering=1)
def log(s): LOG.write("%.1f %s\n"%(time.time(),s))
def d11(n=6):
    vals=[]
    for _ in range(n):
        v=rd("d11")
        if v: vals.append(float(v))
        time.sleep(0.04)
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
def seth(target):
    for _ in range(25):
        dh=wrap(target-havg())
        if abs(dh)<7: return True
        if rd("d0")=="1": motors(-4,-4); time.sleep(0.6); motors(0,0); time.sleep(0.3); continue
        rate=4; t=min(abs(dh)/(2.2*rate),1.2)
        if dh>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.12)
    return False
t0=time.time()
probe=[0,45,-45,90,-90,135,-135,180]
offs=[0,30,-30,60,-60,90,-90,120,-120,150,-150,180]
best_off=None; best_slope=None
oi=0
mode="probe"
v_prev=d11(8)
t_probe=0
cur_off=0
while time.time()-t0<4200:
    if rd("d0")=="1":
        log("BUMP d11=%.3f"%d11(4)); motors(-5,-5); time.sleep(0.9); motors(0,0); time.sleep(0.4); continue
    v=d11(4)
    if v<0.22:
        log("CLOSE v=%.3f - listening"%v); motors(0,0)
        time.sleep(3)
        r=rd("d10",timeout=1.0)
        if r: log("RX!! %s"%r)
        continue
    if mode=="probe":
        off=offs[oi%len(offs)]
        target=(havg()+off)%360 if oi<len(offs) else (base+off)%360
        if oi==0: base=havg()
        seth(target)
        a=d11(8)
        motors(9,9); tp=time.time()
        while time.time()-tp<9:
            time.sleep(1.0)
            if rd("d0")=="1": break
        motors(0,0); time.sleep(0.4)
        b=d11(8)
        slope=(b-a)/9.0
        log("PROBE off=%d h=%.0f d11 %.3f->%.3f slope=%+.4f"%(off,havg(),a,b,slope))
        if best_slope is None or slope<best_slope-0.003:
            best_slope=slope; best_off=off
        oi+=1
        if oi>=len(offs):
            if best_off is None: best_off=0
            base=(base+best_off)%360
            log("LOCK base=%.0f slope=%.4f"%(base,best_slope))
            mode="run"; t_run=time.time(); best_slope=None; best_off=None; oi=0
        continue
    else:
        # run along base, re-evaluate every 20s
        l=lid()
        if l and clean(l)[8]<0.4:
            # avoid: sidestep turn left-hand
            th_t=(havg()-85)%360; seth(th_t); log("AVOID"); continue
        seth(base)
        motors(9,9); time.sleep(2.0)
        motors(0,0); time.sleep(0.3)
        v2=d11(6)
        log("RUN d11=%.3f"%v2)
        if time.time()-t_run>20:
            mode="probe"; t_run=time.time()
motors(0,0)
log("MEET END")
