import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
def d11(n=5):
    vals=[]
    for _ in range(n):
        v=rd("d11")
        if v: vals.append(float(v))
        time.sleep(0.05)
    return sum(vals)/len(vals) if vals else 9
def lid(): 
    for _ in range(3):
        l=lidar()
        if l: return l
        time.sleep(0.05)
def havg(n=4):
    vals=[heading() for _ in range(n)]
    s=sum(math.sin(math.radians(v)) for v in vals)/n
    c=sum(math.cos(math.radians(v)) for v in vals)/n
    return math.degrees(math.atan2(s,c))%360
LOG=open("/bot/src/home.log","a",buffering=1)
motors(0,0); time.sleep(0.5)
cur=d11(8)
h=havg()
best_h=None; best_v=cur
t0=time.time()
mode="seek"   # seek: try headings; run: drive best heading
cand=[0,45,-45,90,-90,135,-135,180]
ci=0
base_h=h
while time.time()-t0<2400:
    l=lid()
    b8=l[8] if l else 0
    if rd("d0")=="1":
        LOG.write("%.1f BUMP v=%.3f h=%.0f\n"%(time.time(),cur,havg()))
        motors(-4,-4); time.sleep(0.7); motors(0,0); time.sleep(0.3)
        # rotate 60 toward open side and re-seek
        mode="seek"; base_h=(havg()+ (60 if (l and l[3]>l[13]) else -60))%360
        continue
    if mode=="run":
        # steer to keep heading near base_h, drive
        h=havg(); dh=(base_h-h+540)%360-180
        lft=(l[6] if l else 1); rgt=(l[10] if l else 1)
        lb=rb=5
        if dh>4: lb=3; rb=5
        elif dh<-4: lb=5; rb=3
        else: lb=rb=6
        if b8<0.35: mode="seek"; motors(0,0); time.sleep(0.2); continue
        motors(lb,rb); time.sleep(0.5)
        v=d11(4)
        if v<cur-0.004 or time.time()-t_track>12:
            # track progress
            pass
        if int(time.time())%3==0:
            LOG.write("%.1f RUN v=%.3f h=%.0f b8=%.2f d0=%s d5=%s d3=%s\n"%(time.time(),v,havg(),b8,rd("d0"),rd("d5"),rd("d3")))
        if v<cur: cur=v; t_track=time.time()
        if cur<0.02: 
            LOG.write("GOAL? v=%.3f d3=%s d5=%s\n"%(cur,rd("d3"),rd("d5")))
            motors(0,0); break
        continue
    # seek: test candidate heading offsets from base_h
    if ci>=len(cand):
        ci=0
    target=(base_h+cand[ci])%360
    # rotate to target
    for _ in range(20):
        h=havg(); dh=(target-h+540)%360-180
        if abs(dh)<8: break
        rate=4; t=min(abs(dh)/(2.2*rate),1.2)
        if dh>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.15)
    v=d11(6)
    LOG.write("%.1f SEEK off=%d h=%.0f v=%.3f (cur=%.3f) d5=%s\n"%(time.time(),cand[ci],havg(),v,cur,rd("d5")))
    if v<best_v-0.003 or (best_h is None):
        if v<best_v: best_v=v
        best_h=target
        if v<cur-0.01: 
            base_h=target; cur=v; mode="run"; t_track=time.time()
            LOG.write("%.1f LOCK h=%.0f v=%.3f\n"%(time.time(),base_h,v))
            continue
    ci+=1
    if ci>=len(cand):
        # none better: go with best_h anyway
        base_h=best_h; mode="run"; t_track=time.time()
motors(0,0)
LOG.write("HOME END v=%.3f\n"%cur)
