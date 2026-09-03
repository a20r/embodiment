import time, math, json, sys
from robot import Robot

r=Robot()
time.sleep(0.5)
LOG=open("/memory/explore.log","a",buffering=1)
def log(*a):
    LOG.write(" ".join(str(x) for x in a)+"\n")

# pose estimate by dead reckoning
x,y=0.0,0.0
SPEED_K=0.0028
last=time.time()
cur_l,cur_r=0,0
def motors(l,rr):
    global cur_l,cur_r
    r.motors(l,rr); cur_l,cur_r=l,rr
def upd():
    global x,y,last
    now=time.time(); dt=now-last; last=now
    v=SPEED_K*(cur_l+cur_r)/2
    h=math.radians(r.hdg or 0)
    x+=v*dt*math.cos(h); y+=v*dt*math.sin(h)

def beam(i):
    l=r.lidar
    v=l[i%16]
    if v<0: v=2.5
    return v

TARGET=0.30
BASE=60
t0=time.time()
lastlog=0
lastbeacon=0
state="follow"
seen_goal=False
try:
  while time.time()-t0 < 55:
    upd()
    st=r.stat
    # beacon + rx
    if time.time()-lastbeacon>2:
        motd=f"PING A x={x:.2f} y={y:.2f} goal={st.get('goal')} here={st.get('here')}"
        r.send(motd); lastbeacon=time.time()
    for m in r.get_rx():
        log("RX:",m)
        print("RX:",m,flush=True)
    if st.get('here')=='1' or st.get('goal')=='1':
        log("STATUS CHANGE",st, "pose",x,y)
        print("STATUS",st,flush=True)
    front=min(beam(15),beam(0),beam(1))
    side=min(beam(4), beam(3)*0.92)
    if r.flags.get('d5')=='1':
        motors(-60,-60); time.sleep(0.6); motors(0,0)
        log("bump backup")
        continue
    if front<0.25:
        # turn away from wall side (decrease hdg) until front clear
        motors(-25,25)
        time.sleep(0.15)
    else:
        err=side-TARGET
        s=max(-25,min(25, 120*err))
        # slow if front close
        b=BASE if front>0.5 else 30
        motors(b+s, b-s)
        time.sleep(0.1)
    if time.time()-lastlog>2:
        lastlog=time.time()
        log(f"t={time.time()-t0:.0f} pose=({x:.2f},{y:.2f}) hdg={r.hdg:.0f} front={front:.2f} side={side:.2f} stat={st}")
finally:
    motors(0,0)
    log("explore session end pose",x,y)
