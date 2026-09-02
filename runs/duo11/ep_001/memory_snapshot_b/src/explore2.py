import time, math, json, os
from robot import Robot

r=Robot(); time.sleep(0.5)
LOG=open("/memory/explore.log","a",buffering=1)
def log(*a): LOG.write(" ".join(str(x) for x in a)+"\n")

pose_file="/memory/pose.json"
x,y=0.0,0.0
if os.path.exists(pose_file):
    try:
        d=json.load(open(pose_file)); x,y=d["x"],d["y"]
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

TARGET=0.30; BASE=70
t0=time.time(); lastlog=0; lastbeacon=0; lastsave=0
try:
  while True:
    upd()
    st=r.stat
    now=time.time()
    if now-lastbeacon>2:
        r.send(f"PING A x={x:.2f} y={y:.2f}"); lastbeacon=now
    for m in r.get_rx():
        log("RX:",m)
    if st.get('here')=='1' or st.get('goal')=='1':
        log("STATUSCHANGE",json.dumps(st),"pose",round(x,2),round(y,2))
    if now-lastsave>5:
        json.dump({"x":x,"y":y},open(pose_file,"w")); lastsave=now
    front=min(beam(15),beam(0),beam(1))
    side=min(beam(4), beam(3)*0.92)
    if r.flags.get('d5')=='1':
        motors(-60,-60); time.sleep(0.6); motors(0,0); log("bump")
        continue
    if front<0.25:
        motors(-25,25); time.sleep(0.15)
    else:
        err=side-TARGET
        s=max(-25,min(25,120*err))
        b=BASE if front>0.5 else 30
        motors(b+s,b-s); time.sleep(0.1)
    if now-lastlog>3:
        lastlog=now
        log(f"t={now-t0:.0f} pose=({x:.2f},{y:.2f}) hdg={r.hdg:.0f} f={front:.2f} s={side:.2f} goal={st.get('goal')} here={st.get('here')}")
finally:
    motors(0,0); log("end pose",x,y)
    json.dump({"x":x,"y":y},open(pose_file,"w"))
