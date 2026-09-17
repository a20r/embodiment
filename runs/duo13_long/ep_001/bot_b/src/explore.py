import time, sys, math, json
sys.path.insert(0,"/bot/src")
from robot import *

# Dead-reckoning explorer: drive toward most open direction, avoid close walls.
# Log pose estimate. Encoders: L=d9 (motor d1), R=d6 (motor d7).
state = {"x":0.0,"y":0.0,"th":None, "el":0.0,"er":0.0, "t":0}
def save():
    with open("/bot/src/pose.json","w") as f: json.dump(state,f)

el=float(rd("d9") or 0); er=float(rd("d6") or 0)
last=el,er
K = 0.01  # meters per tick guess (to calibrate)
# goal: find beam with max openness among valid, prefer near beam8
def choose_dir():
    l=lidar()
    if not l: return None
    best=None; bestscore=-1
    for i in range(16):
        d=l[i]
        if d is None: continue
        if d<0: d=1.5  # no return = open
        # score: distance, prefer beams near forward
        ang=abs(i-8)*22.5
        score=d - ang/180.0*0.8
        if score>bestscore: bestscore=score; best=(i,d)
    return best

t_end=time.time()+240
t0=time.time()
log=open("/bot/src/explore.log","w")
while time.time()<t_end:
    # sense
    h=heading()
    if state["th"] is None: state["th"]=h
    # update pose from encoders
    el=float(rd("d9") or last[0]); er=float(rd("d6") or last[1])
    dl=(el-last[0])*K; dr=(er-last[1])*K
    last=el,er
    ds=(dl+dr)/2.0; dth=(dr-dl)/0.2  # assume track 0.2m
    state["x"]+=ds*math.cos(math.radians(state["th"]))
    state["y"]+=ds*math.sin(math.radians(state["th"]))
    state["th"]=(state["th"]+dth)%360
    # control
    ch=choose_dir()
    l=lidar() or []
    b8=l[8] if len(l)>8 else 0
    b7=l[7] if len(l)>7 else 0
    b9=l[9] if len(l)>9 else 0
    l7=b7 if b7>0 else 2; l8=b8 if b8>0 else 2; l9=b9 if b9>0 else 2
    turn=0; fwd=3
    if ch:
        i,d=ch
        err=(i-8)*22.5
        if err>11: turn=1
        elif err<-11: turn=-1
    # wall avoidance
    if l8<0.25 or l7<0.15: turn=+1  # veer right(CW?+) hmm
    if l9<0.15: turn=-1
    if l8<0.12: fwd=0
    if turn>0: motors(2,-2); log.write("SPIN+ t=%.0f h=%.0f\n"%(time.time()-t0,h))
    elif turn<0: motors(-2,2); log.write("SPIN- t=%.0f h=%.0f\n"%(time.time()-t0,h))
    else: motors(fwd,fwd)
    if int((time.time()-t0))%10==0:
        log.write("t=%.0f h=%.0f pose=(%.2f,%.2f,%.0f) ch=%s l7=%.2f l8=%.2f l9=%.2f d3=%s d0=%s d5=%s d11=%s\n"%(
            time.time()-t0,h,state["x"],state["y"],state["th"],ch,l7,l8,l9,rd("d3"),rd("d0"),rd("d5"),rd("d11")))
        log.flush(); save()
    time.sleep(0.25)
motors(0,0)
log.close(); save()
