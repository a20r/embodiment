import sys, time, math, json
sys.path.insert(0,'/bot/src')
from robot import R
MPT=0.000503
# initialize from existing pose.json if fresh
x=y=0.0
try:
    j=json.load(open('/memory/pose.json'))
    x,y=j['x'],j['y']
    print(f"posetrack: start from pose.json ({x:.2f},{y:.2f})",flush=True)
except Exception:
    print("posetrack: fresh origin here",flush=True)
r=R(); r.motors(0,0)
prev=None
f=open('/memory/posetrack.log','a',buffering=1)
f.write(f"[{time.time():.0f}] posetrack start ({x:.2f},{y:.2f})\n")
t0=time.time(); lastw=0
while True:
    e=r.enc(); h=r.heading()
    ok = e and e[0] is not None and e[1] is not None
    if ok and prev and prev[0] is not None and h is not None:
        dl=e[0]-prev[0]; dr=e[1]-prev[1]
        d=(dl+dr)/2.0*MPT
        if abs(d)<0.2:  # sane
            x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
    if ok: prev=e
    if time.time()-lastw>0.8:
        lastw=time.time()
        try:
            json.dump({'x':x,'y':y,'t':time.time()},open('/memory/pose.json','w'))
            open('/memory/heartbeat','w').write(f"{time.time():.0f} {x:.2f} {y:.2f} m=posetrack\n")
        except Exception: pass
    time.sleep(0.12)
