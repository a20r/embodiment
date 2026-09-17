import time,sys,math,os
sys.path.insert(0,"/bot/src")
from robot import *
L=open("/bot/src/goalseek.log","a",buffering=1)
def log(s): L.write("%.1f %s\n"%(time.time(),s))
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
def flags():
    g=h=0
    for _ in range(4):
        s=rd("d3",timeout=0.03) or ""
        if "goal=1" in s: g=1
        if "here=1" in s: h=1
    return (g,h)
def rotate_to(target):
    for _ in range(40):
        if rd("d0")=="1":
            motors(-5,-5); time.sleep(0.8); motors(0,0); time.sleep(0.4); continue
        dh=wrap(target-havg())
        if abs(dh)<7: return True
        rate=4; t=min(abs(dh)/(2.2*rate),1.0)
        if dh>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.1)
    return False
log("ARMED (passive watch)")
while True:
    g,hh=flags()
    if g or hh: break
    time.sleep(0.1)
log("TRIGGER g=%d hh=%d"%(g,hh))
open("/bot/src/SEEKING","w").write("seeking\n")
os.system("/bot/src/killhelper.sh explore6.py")
open("/bot/src/GOALALERT.txt","a").write("%.1f TRIGGER g=%d hh=%d\n"%(time.time(),g,hh))
# Phase 1: directional scan
scan=[]; t0=time.time(); h0=havg()
for k in range(16):
    rotate_to((h0+k*22.5)%360)
    on=0
    for _ in range(8):
        g2,_=flags(); on+=g2; time.sleep(0.07)
    l=lid()
    scan.append((k*22.5,on,l))
    log("SCAN %d on=%d"%(k*22.5,on))
with open("/bot/src/GOALSCAN.txt","w") as f:
    for a,on,l in scan: f.write("%.0f %d %s\n"%(a,on,l))
bestb=None; beston=0
for a,on,_ in scan:
    if on>beston: beston=on; bestb=a
if beston>0:
    log("BEST bearing %d on=%d"%(bestb,beston))
    rotate_to((h0+bestb)%360)
# Phase 2: drive toward flag; reverse-hunt on loss
th=havg(); last_on=None; ever=False
while time.time()-t0<2400:
    if rd("d0")=="1":
        motors(-5,-5); time.sleep(0.9); motors(0,0); time.sleep(0.4); th=None; continue
    g,hh=flags()
    if hh==1:
        log("HERE=1 !"); motors(0,0)
        open("/bot/src/GOALALERT.txt","a").write("%.1f HERE1\n"%time.time())
        while True:
            wr("d8","B HERE ON GOAL - COME NOW")
            time.sleep(2)
            s3=rd("d3",timeout=0.2) or ""
            open("/bot/src/GOALALERT.txt","a").write("%.1f HOLD %s\n"%(time.time(),s3))
            if "here=0" in s3:
                log("lost here - nudge")
                for _ in range(4):
                    motors(5,5); time.sleep(0.4); motors(0,0); time.sleep(0.2)
                    if "here=1" in (rd("d3",timeout=0.2) or ""): break
    if g==1:
        ever=True; last_on=time.time()
    elif ever and last_on is not None and time.time()-last_on>1.0 and time.time()-last_on<1.5:
        pass
    # lost handling
    if ever and (last_on is None or time.time()-last_on>8):
        if time.time()-last_on<8.6:
            log("LOST -> reverse hunt")
            motors(-8,-8); t_r=time.time(); got=False
            while time.time()-t_r<6:
                g2,_=flags()
                if g2==1: got=True; break
                time.sleep(0.1)
            motors(0,0); time.sleep(0.3); th=None
            if got:
                log("REVERSE FOUND")
                last_on=time.time(); ever=True
                motors(5,5); t_c=time.time()
                while time.time()-t_c<8:
                    g2,_=flags()
                    if g2==0: break
                    time.sleep(0.1)
                motors(0,0); time.sleep(0.2); th=None
                continue
            else:
                last_on=time.time()-30  # force rescan next
        else:
            log("RESCAN")
            bestb2=None; beston2=0; h0=havg()
            for k in range(16):
                rotate_to((h0+k*22.5)%360)
                on=0
                for _ in range(6):
                    g2,_=flags(); on+=g2; time.sleep(0.06)
                if on>beston2: beston2=on; bestb2=k*22.5
            if beston2>0:
                log("RESCAN best %d"%(bestb2,))
                rotate_to((h0+bestb2)%360); th=havg(); last_on=time.time()
            else:
                log("RESCAN nothing - turn 90")
                rotate_to((havg()+90)%360); th=havg(); last_on=time.time()-4
        continue
    l=lid()
    if l:
        b=clean(l); b8=b[8]
        if b8<0.35:
            cands=[(i,b[i]**2) for i in range(16) if b[i]>0.4 and i!=8]
            if not cands: cands=[(1,1),(15,1)]
            tot=sum(w for _,w in cands); r=time.time()%tot; acc=0; pick=cands[0][0]
            for i,w in cands:
                acc+=w
                if r<=acc: pick=i; break
            log("AVOID beam %d"%(pick,))
            rotate_to((havg()+(pick-8)*22.5)%360); th=havg(); continue
        dl=min(b[4],b[5]); dr=min(b[11],b[12])
        if dl<0.2: th=havg()+7
        elif dr<0.15: th=havg()-6
    dh=wrap(th-havg())
    lb=11+int(round(0.09*dh)); rb=11-int(round(0.09*dh))
    motors(max(4,min(13,lb)),max(4,min(13,rb)))
    time.sleep(0.2)
motors(0,0)
log("GOALSEEK timeout end")
