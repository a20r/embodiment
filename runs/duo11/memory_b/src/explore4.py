import time, math, json, os, random
from robot import Robot

r=Robot(); time.sleep(0.5)
LOG=open("/memory/explore.log","a",buffering=1)
def log(*a): LOG.write(" ".join(str(x) for x in a)+"\n")
log("=== explore4 start ===")

pose_file="/memory/pose.json"
x,y=0.0,0.0
try: d=json.load(open(pose_file)); x,y=d["x"],d["y"]
except: pass
SPEED_K=0.0028
last=time.time(); cur_l=cur_r=0
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
    v=r.lidar[i%16]
    return 2.5 if v<0 else v
lastbeacon=0
def housekeeping():
    global lastbeacon
    st=r.stat
    for m in r.get_rx(): log("RX:",m)
    if st.get('here')=='1' or st.get('goal')=='1':
        log("STATUSCHANGE",json.dumps(st),f"pose=({x:.2f},{y:.2f})")
    if time.time()-lastbeacon>2:
        r.send(f"PING A x={x:.2f} y={y:.2f}"); lastbeacon=time.time()

TARGET=0.28; BASE=70
side_sign=-1  # 1: follow wall on beam4 side; -1: beam12 side
hist=[]  # (t,x,y)
t0=time.time(); lastlog=0; lastsave=0
try:
  while True:
    upd(); housekeeping()
    now=time.time()
    hist.append((now,x,y))
    while hist and hist[0][0]<now-70: hist.pop(0)
    # loop escape: displacement over 60s < 0.7
    if hist and now-hist[0][0]>60:
        ot,ox,oy=hist[0]
        if math.hypot(x-ox,y-oy)<0.7:
            side_sign=-side_sign
            log(f"LOOP ESCAPE switch side to {side_sign} at ({x:.2f},{y:.2f})")
            hist=[]
    if now-lastsave>5:
        json.dump({"x":x,"y":y},open(pose_file,"w")); lastsave=now
    if now-lastlog>3:
        lastlog=now
        log(f"t={now-t0:.0f} pose=({x:.2f},{y:.2f}) hdg={r.hdg:.0f} ss={side_sign}")
    if r.flags.get('d5')=='1':
        motors(-60,-60); time.sleep(0.5); motors(0,0); log("bump")
        continue
    front=min(beam(15),beam(0),beam(1))
    if side_sign==1:
        side=min(beam(4),beam(3)*0.92)
    else:
        side=min(beam(12),beam(13)*0.92)
    if front<0.25:
        motors(-25*side_sign,25*side_sign); time.sleep(0.15)
    else:
        err=side-TARGET
        s=side_sign*max(-25,min(25,120*err))
        b=BASE if front>0.5 else 32
        motors(b+s,b-s); time.sleep(0.1)
finally:
    motors(0,0); json.dump({"x":x,"y":y},open(pose_file,"w")); log("end",x,y)
