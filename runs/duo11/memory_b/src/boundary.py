import time, math, json, sys, os, threading
from robot import Robot

SIGN=int(sys.argv[1]) if len(sys.argv)>1 else 1  # +1: wall kept at (heading-90) i.e. travel with wall on that side... we define below
rssi_vals=[]
def rssi_thread():
    fd=os.open("/dev/robot/d11",os.O_RDONLY|os.O_NONBLOCK)
    buf=b""
    while True:
        try:
            d=os.read(fd,4096)
            if d:
                buf+=d
                parts=buf.split(b"\n"); buf=parts[-1]
                for l in parts[:-1]:
                    try: rssi_vals.append((time.time(),float(l)))
                    except: pass
            else: time.sleep(0.01)
        except BlockingIOError: time.sleep(0.01)
threading.Thread(target=rssi_thread,daemon=True).start()
def rssi(win=2.0):
    now=time.time()
    vs=[v for t,v in rssi_vals if now-t<win]
    return sum(vs)/len(vs) if vs else 0.0
r=Robot(); time.sleep(0.5)
LOG=open("/memory/explore.log","a",buffering=1)
def log(*a): LOG.write(" ".join(str(x) for x in a)+"\n")
log(f"=== boundary start sign={SIGN} ===")
d=json.load(open("/memory/pose.json")); x,y=d["x"],d["y"]
last=time.time(); cur=(0,0)
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
def beams():
    l=r.lidar; h=r.hdg
    out=[]
    for i,v in enumerate(l):
        if v<0: v=3.0
        out.append(((h+22.5*i)%360, v))
    return out
lastb=0; lastlog=0
TD=0.45  # target distance to wall
t0=time.time()
try:
  while True:
    upd()
    now=time.time()
    if now-lastb>2:
        r.send(f"PING A x={x:.2f} y={y:.2f}"); lastb=now
    for m in r.get_rx():
        log("RX:",m); print("RX:",m,flush=True)
    bs=beams()
    wa,wd=min(bs,key=lambda t:t[1])  # nearest wall abs bearing & dist
    if r.flags.get('d5')=='1':
        motors(-60,-60); time.sleep(0.5); motors(0,0); log("bump")
        continue
    # desired travel heading: perpendicular to wall bearing
    base_hdg=(wa + SIGN*90) % 360
    # correction: too far -> angle toward wall; too close -> away
    corr=max(-40,min(40, (wd-TD)*100))
    des=(base_hdg - SIGN*corr) % 360
    h=r.hdg
    err=(des-h+180)%360-180
    # front clearance along current heading
    front=min(v for a,v in bs if abs((a-h+180)%360-180)<=35)
    if abs(err)>50:
        s=max(10,min(35,abs(err)*0.5))
        motors(s if err>0 else -s, -s if err>0 else s)
    else:
        b=85 if front>0.8 else (45 if front>0.4 else 25)
        s=max(-20,min(20,err*0.8))
        motors(b+s,b-s)
    if rssi()>1.2:
        log(f"RSSI HIGH {rssi():.3f} at ({x:.2f},{y:.2f}) STOP")
        motors(0,0)
        break
    if now-lastlog>3:
        lastlog=now
        log(f"t={now-t0:.0f} pose=({x:.2f},{y:.2f}) hdg={h:.0f} wall@{wa:.0f}/{wd:.2f} front={front:.2f} rssi={rssi():.4f}")
        json.dump({"x":x,"y":y},open("/memory/pose.json","w"))
    time.sleep(0.08)
finally:
    motors(0,0); json.dump({"x":x,"y":y},open("/memory/pose.json","w"))
