import time, sys, math, json
sys.path.insert(0,"/bot/src")
from robot import *

LOG=open("/bot/src/nav.log","a", buffering=1)
def log(s): LOG.write("%.1f %s\n" % (time.time(), s))

state={"sign":+1, "total_ticks":0, "bumps":0, "pose":[0,0], "elapsed":0}
try: state.update(json.load(open("/bot/src/navstate.json")))
except: pass

el=0.0; er=0.0
def read_enc():
    global el, er
    a=rd("d9"); b=rd("d6")
    if a is not None: el=float(a)
    if b is not None: er=float(b)

def havg(n=4):
    # compass average handling wrap
    vals=[heading() for _ in range(n)]
    s=sum(math.sin(math.radians(v)) for v in vals)/n
    c=sum(math.cos(math.radians(v)) for v in vals)/n
    return (math.degrees(math.atan2(s,c)))%360

def lid():
    for _ in range(3):
        l=lidar()
        if l: return l
        time.sleep(0.05)
    return None

def beams(l):
    # return cleaned: -1 -> 1.6 (open beyond range)
    return [1.6 if (x is None or x<0) else x for x in l]

def choose_dir(bl):
    # score beams: prefer open; penalize angular distance from forward
    best=None; bs=-9
    for i in range(16):
        ang=abs(i-8)*22.5
        sc=bl[i]-ang/22.5*0.12
        if sc>bs: bs=sc; best=i
    return best, bs

def stopped(): motors(0,0)

FWD=6
last_decision=""
last_log=0
t_start=time.time()
read_enc()
e_prev=(el,er)
while time.time()-t_start < 5400:
    loop_t=time.time()
    l=lid()
    if l is None: stopped(); time.sleep(0.2); continue
    bl=beams(l)
    b7,b8,b9=bl[7],bl[8],bl[9]
    h=havg()
    # encoder update
    read_enc()
    dt=(el-e_prev[0], er-e_prev[1]); e_prev=(el,er)
    state["total_ticks"]+= (dt[0]+dt[1])/2
    # bump?
    if rd("d0")=="1":
        state["bumps"]+=1
        log("BUMP h=%.0f b8=%.2f ticks=%d" % (h,b8,state["total_ticks"]))
        motors(-4,-4); time.sleep(0.8); stopped(); time.sleep(0.3)
        # rotate toward most open
        l=lid(); bl=beams(l) if l else bl
        best,_=choose_dir(bl)
        err=(best-8)
        if err!=0:
            t=min(abs(err)*2.4, 12)  # rate4 ~9.2deg/s -> 2.4s per beam
            if (err>0) == (state["sign"]>0): motors(4,-4)
            else: motors(-4,4)
            time.sleep(t); stopped(); time.sleep(0.3)
        read_enc(); e_prev=(el,er)
        continue
    # choose direction
    best,sc=choose_dir(bl)
    err=best-8
    # steering
    if abs(err)>=1:
        # proportional: rotate by err beams worth
        deg=err*22.5*0.45
        rate=4
        t=abs(deg)/ (2.2*rate)
        if (deg>0)==(state["sign"]>0): motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(min(t,1.5)); stopped(); time.sleep(0.15)
        # learn steering sign: re-scan; if best moved away from 8, flip
        l2=lid()
        if l2:
            bl2=beams(l2); best2,_=choose_dir(bl2)
            if abs(best2-8)>abs(err)+0.5 and abs(err)>=2:
                state["sign"]*=-1
                log("FLIP sign -> %d (best %d->%d err %d)" % (state["sign"], best, best2, err))
        read_enc(); e_prev=(el,er)
        continue
    # straight, with soft wall follow
    drive=FWD
    if b8<0.5: drive=3
    if b8<0.3:
        stopped(); time.sleep(0.2); continue
    lft,rgt=bl[5],bl[11]
    bias=0.0
    if lft<0.3 and rgt>0.5: bias=0.8   # wall left: bear right (CW+) if sign>0
    if rgt<0.3 and lft>0.5: bias=-0.8
    if abs(bias)>0.01:
        base=drive
        if state["sign"]>0: motors(base+bias*2 if bias>0 else base, base-bias*2 if bias>0 else base)
        else: motors(base-bias*2 if bias>0 else base, base+bias*2 if bias>0 else base)
    else:
        motors(drive,drive)
    # periodic log
    if time.time()-loop_t>=0:
        pass
    if time.time()-last_log>15:
        last_log=time.time()
        log("h=%.0f b7=%.2f b8=%.2f b9=%.2f best=%d ticks=%d d0=%s d5=%s d3=%s d11=%s" % (h,b7,b8,b9,best,state["total_ticks"],rd("d0"),rd("d5"),rd("d3"),rd("d11")))
        json.dump(state, open("/bot/src/navstate.json","w"))
    time.sleep(0.3)
stopped()
json.dump(state, open("/bot/src/navstate.json","w"))
log("NAV END")
