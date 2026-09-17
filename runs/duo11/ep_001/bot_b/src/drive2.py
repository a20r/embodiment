import time, math, sys, json
from robot import Robot
r=Robot(); time.sleep(0.4)
MAP=open("/memory/map.log","a",buffering=1)
def beam(i):
    v=r.lidar[i%16]
    return 2.5 if v<0 else v
def turn_to(target):
    while True:
        err=(target-r.hdg+180)%360-180
        if abs(err)<5: break
        s=max(6,min(35,abs(err)*0.7))
        r.motors(s if err>0 else -s,-s if err>0 else s)
        time.sleep(0.08)
    r.motors(0,0)
tgt=float(sys.argv[1]); maxt=float(sys.argv[2]) if len(sys.argv)>2 else 50
d=json.load(open("/memory/pose.json")); x,y=d["x"],d["y"]
turn_to(tgt)
last=time.time(); t0=last
lastb=0; dist=0
stop_reason="time"
while time.time()-t0<maxt:
    fr=[beam(15),beam(0),beam(1)]
    front=min(fr)
    if front<0.4 and sorted(fr)[1]<2:  # require 2 beams agree-ish OR confirm
        time.sleep(0.05)
        fr2=[beam(15),beam(0),beam(1)]
        if min(fr2)<0.4:
            stop_reason="wall"; break
    if r.flags.get('d5')=='1':
        stop_reason="bump"; break
    for m in r.get_rx():
        print("RX:",m); MAP.write(f"RX at ({x:.2f},{y:.2f}): {m}\n")
    if time.time()-lastb>2:
        r.send(f"PING A x={x:.2f} y={y:.2f}"); lastb=time.time()
    err=(tgt-r.hdg+180)%360-180
    s=max(-12,min(12,err*0.7))
    b=90 if front>0.8 else 40
    r.motors(b+s,b-s)
    now=time.time()
    v=0.0028*b; h=math.radians(r.hdg)
    x+=v*(now-last)*math.cos(h); y+=v*(now-last)*math.sin(h); dist+=v*(now-last)
    last=now
    time.sleep(0.06)
r.motors(0,0)
json.dump({"x":x,"y":y},open("/memory/pose.json","w"))
mn=min(beam(15),beam(0),beam(1))
print(f"stop={stop_reason} pos=({x:.2f},{y:.2f}) dist={dist:.2f} front={mn:.2f} hdg={r.hdg:.0f} here={r.stat.get('here')} d0={r.flags.get('d0')}")
