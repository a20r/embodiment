import sys, time, json, math
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import motors, stop, turn_by

K_V=0.0026  # m/s per cmd unit (avg of wheels)
LOG=open('/tmp/map.log','a')
def log(**kw):
    kw['t']=round(time.time(),2); LOG.write(json.dumps(kw)+"\n"); LOG.flush()

x=y=0.0
if len(sys.argv)>3: x=float(sys.argv[2]); y=float(sys.argv[3])
FWD=160
t_end=time.time()+float(sys.argv[1])
last_ping=0; last_t=time.time()
cl=cr=0
try:
  while time.time()<t_end:
    now=time.time(); dt=now-last_t; last_t=now
    c=compass()
    if c is not None:
        v=K_V*(cl+cr)/2.0
        x+=v*math.cos(math.radians(c))*dt
        y+=v*math.sin(math.radians(c))*dt
    if now-last_ping>2:
        write_port("d8","PING A x=%.2f y=%.2f"%(x,y)); last_ping=now
    d9=read_port("d9")
    if d9 and 'goal=0' not in d9:
        log(event='GOAL', d9=d9, x=x, y=y); break
    L=lidar()
    if L is None: continue
    s=read_port("d6"); s=float(s) if s else None
    log(x=round(x,3), y=round(y,3), c=c, s=s, L=L)
    fl=[v for v in (L[0],L[1],L[15]) if v>0]
    front=min(fl) if fl else 9
    bump=read_port("d0")=='1'
    if bump or front<0.35:
        motors(-120,-120); time.sleep(0.3); stop()
        if c is not None:
            x-=K_V*120*0.3*math.cos(math.radians(compass() or c))  # rough backup correction
            y-=K_V*120*0.3*math.sin(math.radians(compass() or c))
        L2=lidar() or L
        leftd=L2[4] if L2[4]>0 else 9; rightd=L2[12] if L2[12]>0 else 9
        turn_by(80 if leftd>rightd else -80)
        cl=cr=0
        continue
    left=L[4] if L[4]>0 else 9
    err=max(min(left-0.35,0.5),-0.5)
    steer=err*150
    if left>1.3:
        cl,cr=FWD-60,FWD+60
    else:
        cl,cr=FWD-steer,FWD+steer
    # slow if front close
    if front<0.7:
        f=max(front-0.3,0.05)/0.4
        cl*=f; cr*=f
        cl=max(cl,40); cr=max(cr,40)
    motors(cl,cr)
    time.sleep(0.1)
finally:
  stop(); log(event='END', x=x, y=y)
