import time, math, json
from robot import Robot
r=Robot(); time.sleep(0.5)
LOG=open("/memory/spiral.log","a",buffering=1)
def log(*a): LOG.write(" ".join(str(x) for x in a)+"\n")
d=json.load(open("/memory/pose.json")); x,y=d["x"],d["y"]
log(f"=== spiral start ({x:.2f},{y:.2f}) ===")
cur=(0,0); last=time.time()
def motors(l,rr):
    global cur
    r.motors(l,rr); cur=(l,rr)
def upd():
    global x,y,last
    now=time.time()
    v=0.0028*(cur[0]+cur[1])/2
    h=math.radians(r.hdg or 0)
    x+=v*(now-last)*math.cos(h); y+=v*(now-last)*math.sin(h)
    last=now
def beam_pts():
    l=r.lidar; h=r.hdg
    for i,v in enumerate(l):
        if v<0 or v>2.9: continue
        a=math.radians((h+22.5*i)%360)
        yield (x+v*math.cos(a), y+v*math.sin(a), v, (h+22.5*i)%360)
BBX=(-2.5,6.0,-7.0,3.0)  # known structure bbox x0,x1,y0,y1
def known(px,py):
    return BBX[0]<=px<=BBX[1] and BBX[2]<=py<=BBX[3]
def turn_to(t):
    while True:
        upd()
        err=(t-r.hdg+180)%360-180
        if abs(err)<6: break
        s=max(8,min(35,abs(err)*0.6))
        motors(s if err>0 else -s,-s if err>0 else s)
        time.sleep(0.08)
    motors(0,0)
lastb=0; lastlog=0
def housekeeping(tag):
    global lastb,lastlog
    now=time.time()
    if now-lastb>2:
        r.send(f"PING A x={x:.2f} y={y:.2f}"); lastb=now
    for m in r.get_rx():
        log(f"RX at ({x:.2f},{y:.2f}):",m); print("RXRX",m,flush=True)
    for px,py,v,a in beam_pts():
        if not known(px,py):
            log(f"CONTACT ({px:.2f},{py:.2f}) from ({x:.2f},{y:.2f}) bearing {a:.0f} dist {v:.2f}")
    if now-lastlog>4:
        lastlog=now
        log(f"{tag} pose=({x:.2f},{y:.2f}) hdg={r.hdg:.0f}")
        json.dump({"x":x,"y":y},open("/memory/pose.json","w"))
def front():
    l=r.lidar
    return min(2.5 if v<0 else v for v in [l[15],l[0],l[1]])
def leg(hdg,length):
    turn_to(hdg)
    dist=0; lastu=time.time()
    while dist<length:
        upd(); housekeeping(f"leg{hdg:.0f}/{length:.1f} d={dist:.1f}")
        if r.flags.get('d5')=='1':
            motors(-60,-60); time.sleep(0.5); motors(0,0); return "bump"
        f=front()
        if f<0.7: return "wall"
        err=(hdg-r.hdg+180)%360-180
        s=max(-30,min(30,err*1.2))
        b=240 if f>1.5 else 60
        motors(b+s,b-s)
        now=time.time(); dist+=0.0028*b*(now-lastu); lastu=now
        time.sleep(0.05)
    return "done"
# start: move away from structure to SW corner then spiral around bbox center
cx,cy=1.75,-1.85
L=29.0
hdgs=[0,90,180,270]
hi=0
while True:
    res=leg(hdgs[hi%4],L)
    log(f"leg result {res} at ({x:.2f},{y:.2f})")
    if res=="wall":
        # follow around: turn to next heading early
        pass
    hi+=1
    if hi%2==0: L+=5.0
