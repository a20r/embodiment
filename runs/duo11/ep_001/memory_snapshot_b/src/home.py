import time, math, json, sys, os, threading
from robot import Robot
# rssi reader thread
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
def rssi(win=1.5):
    now=time.time()
    vs=[v for t,v in rssi_vals if now-t<win]
    return sum(vs)/len(vs) if vs else None
r=Robot(); time.sleep(0.5)
d=json.load(open("/memory/pose.json")); x,y=d["x"],d["y"]
LOG=open("/memory/home.log","a",buffering=1)
def log(*a):
    s=" ".join(str(i) for i in a)
    LOG.write(s+"\n"); print(s,flush=True)
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
def turn_to(t):
    while True:
        upd()
        err=(t-r.hdg+180)%360-180
        if abs(err)<6: break
        s=max(8,min(40,abs(err)*0.7))
        motors(s if err>0 else -s,-s if err>0 else s)
        time.sleep(0.07)
    motors(0,0)
def beam(i):
    v=r.lidar[i%16]
    return 3.0 if v<0 else v
def front():
    return min(beam(15),beam(0),beam(1))
def drive(hdg,dist,fast=240):
    turn_to(hdg)
    done=0; lastu=time.time(); lastb=0
    while done<dist:
        upd()
        if time.time()-lastb>2:
            r.send(f"PING A"); lastb=time.time()
        for m in r.get_rx(): log("RX:",m)
        if r.flags.get('d5')=='1':
            motors(-60,-60); time.sleep(0.5); motors(0,0); return "bump",done
        f=front()
        if f<0.7: return "wall",done
        err=(hdg-r.hdg+180)%360-180
        s=max(-30,min(30,err*1.2))
        b=fast if f>1.5 else 60
        motors(b+s,b-s)
        now=time.time(); done+=0.0028*b*(now-lastu); lastu=now
        time.sleep(0.05)
    motors(0,0)
    return "done",done
if __name__=="__main__":
    hdg=float(sys.argv[1]); dist=float(sys.argv[2])
    seg=3.0
    while dist>0:
        res,dn=drive(hdg,min(seg,dist))
        dist-=dn
        upd(); motors(0,0)
        time.sleep(1.0)
        log(f"pose=({x:.2f},{y:.2f}) rssi={rssi():.4f} res={res}")
        json.dump({"x":x,"y":y},open("/memory/pose.json","w"))
        if res in ("wall","bump"): break
