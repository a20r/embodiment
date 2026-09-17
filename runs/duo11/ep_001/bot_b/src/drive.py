import time, math, sys, json
from robot import Robot
r=Robot(); time.sleep(0.4)
def beam(i):
    v=r.lidar[i%16]
    return 2.5 if v<0 else v
def turn_to(target):
    while True:
        err=(target-r.hdg+180)%360-180
        if abs(err)<5: break
        s=max(5,min(35,abs(err)*0.7))
        r.motors(s if err>0 else -s,-s if err>0 else s)
        time.sleep(0.08)
    r.motors(0,0)
tgt=float(sys.argv[1]); maxt=float(sys.argv[2]) if len(sys.argv)>2 else 20
d=json.load(open("/memory/pose.json")); x,y=d["x"],d["y"]
turn_to(tgt)
last=time.time(); t0=last; lasthere=r.stat.get('here')
dist=0
while time.time()-t0<maxt:
    front=min(beam(15),beam(0),beam(1))
    if front<0.28 or r.flags.get('d5')=='1': break
    hh=r.stat.get('here')
    if hh!=lasthere:
        print(f"here {lasthere}->{hh} at ({x:.2f},{y:.2f}) dist={dist:.2f}")
        lasthere=hh
    err=(tgt-r.hdg+180)%360-180
    s=max(-12,min(12,err*0.7))
    # gentle centering
    lft=beam(12); rgt=beam(4)
    if rgt<0.25: s-=8
    if lft<0.25: s+=8
    b=75 if front>0.5 else 35
    r.motors(b+s,b-s)
    now=time.time()
    v=0.0028*b; h=math.radians(r.hdg)
    x+=v*(now-last)*math.cos(h); y+=v*(now-last)*math.sin(h); dist+=v*(now-last)
    last=now
    time.sleep(0.06)
r.motors(0,0)
json.dump({"x":x,"y":y},open("/memory/pose.json","w"))
print(f"pos=({x:.2f},{y:.2f}) dist={dist:.2f} front={min(beam(15),beam(0),beam(1)):.2f} here={r.stat.get('here')} hdg={r.hdg:.0f}")
