import time, math, json, os, random
from robot import Robot

r=Robot(); time.sleep(0.5)
LOG=open("/memory/explore.log","a",buffering=1)
def log(*a): LOG.write(" ".join(str(x) for x in a)+"\n")
log("=== explore3 start ===")

pose_file="/memory/pose.json"
x,y=0.0,0.0
if os.path.exists(pose_file):
    try: d=json.load(open(pose_file)); x,y=d["x"],d["y"]
    except: pass
visited={}  # cell -> count
CELL=0.3
def vkey(px,py): return f"{int(math.floor(px/CELL))},{int(math.floor(py/CELL))}"
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
    visited[vkey(x,y)]=visited.get(vkey(x,y),0)+1
def beam(i):
    v=r.lidar[i%16]
    return 2.5 if v<0 else v
def comms():
    st=r.stat
    for m in r.get_rx(): log("RX:",m)
    if st.get('here')=='1' or st.get('goal')=='1':
        log("STATUSCHANGE",json.dumps(st),"pose",round(x,2),round(y,2))
lastbeacon=0
def beacon():
    global lastbeacon
    if time.time()-lastbeacon>2:
        r.send(f"PING A x={x:.2f} y={y:.2f}"); lastbeacon=time.time()

def turn_to(target):
    while True:
        upd(); comms(); beacon()
        h=r.hdg
        err=(target-h+180)%360-180
        if abs(err)<6: break
        s=max(6,min(30,abs(err)*0.6))
        motors(s if err>0 else -s, -s if err>0 else s)
        time.sleep(0.08)
    motors(0,0)

def choose_dir():
    # score each beam direction by openness + novelty
    h=r.hdg; l=[beam(i) for i in range(16)]
    best=None;bestsc=-1
    for i in range(16):
        d=l[i]
        if d<0.35: continue
        ang=math.radians((h+22.5*i)%360)
        reach=min(d-0.15,1.2)
        # novelty: sample cells along ray
        nov=0; n=0
        for f in (0.3,0.6,0.9,1.2):
            if f>reach: break
            cx,cy=x+f*math.cos(ang), y+f*math.sin(ang)
            nov += 1.0/(1+visited.get(vkey(cx,cy),0)); n+=1
        sc=(nov if n else 0.1)+0.3*min(d,2.0)+random.uniform(0,0.15)
        if sc>bestsc: bestsc=sc; best=(h+22.5*i)%360
    return best

t0=time.time(); lastlog=0; lastsave=0
try:
  while True:
    upd(); comms(); beacon()
    now=time.time()
    if now-lastsave>5:
        json.dump({"x":x,"y":y},open(pose_file,"w")); lastsave=now
    if now-lastlog>3:
        lastlog=now
        log(f"t={now-t0:.0f} pose=({x:.2f},{y:.2f}) hdg={r.hdg:.0f} f={beam(0):.2f} cells={len(visited)}")
    if r.flags.get('d5')=='1':
        motors(-60,-60); time.sleep(0.5); motors(0,0); log("bump")
        tgt=choose_dir()
        if tgt is not None: turn_to(tgt)
        continue
    front=min(beam(15),beam(0),beam(1))
    if front<0.30:
        motors(0,0)
        tgt=choose_dir()
        if tgt is None:
            motors(-50,-50); time.sleep(0.8); motors(0,0)
        else:
            turn_to(tgt)
        continue
    # drive straight w/ mild centering: steer to balance side beams
    lft=min(beam(12),beam(13)*0.92)   # hdg-90 side is beam 12
    rgt=min(beam(4),beam(3)*0.92)
    s=0
    if rgt<0.18: s=-12
    elif lft<0.18: s=12
    b=70 if front>0.5 else 35
    motors(b+s,b-s)
    time.sleep(0.1)
finally:
    motors(0,0); json.dump({"x":x,"y":y},open(pose_file,"w")); log("end pose",x,y)
