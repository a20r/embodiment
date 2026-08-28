import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import motors, stop, turn_by

LOG=open('/tmp/wf.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+"\n"); LOG.flush()

def hot():
    d7=read_port("d7"); d9=read_port("d9")
    if d7 and d7.strip() not in ('0',''):
        log(event='D7',d7=d7); print("D7 HOT",d7,flush=True); return True
    if d9 and 'goal=' in d9 and 'goal=0' not in d9:
        log(event='GOALFLAG',d9=d9); print("GOALFLAG",d9,flush=True); return True
    return False

t_end=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 600)
last_ping=0; last_log=0
try:
  while time.time()<t_end:
    if hot(): break
    now=time.time()
    if now-last_ping>3:
        s=read_port("d6")
        write_port("d8",f"B wallfollowing hunt for goal. s={s}")
        last_ping=now
    L=lidar()
    if L is None: continue
    fl=[v for v in (L[0],L[1],L[15]) if v>0]
    front=min(fl) if fl else 9
    left=L[4] if L[4]>0 else 9
    lf=L[2] if L[2]>0 else 9   # left-front diagonal
    bump=read_port("d0")=='1'
    if now-last_log>1:
        log(c=compass(), s=read_port("d6"), f=round(front,2)); last_log=now
    if bump:
        motors(-120,-120); time.sleep(0.35); stop()
        turn_by(-80)
        continue
    if front<0.24:
        stop(); turn_by(-85)   # turn right (compass-)
        continue
    err=left-0.23
    err=max(min(err,0.3),-0.3)
    steer=err*220
    if lf<0.2: steer-=35   # inner corner guard
    base=150 if front>0.6 else 90
    if left>0.8:  # opening on left: arc left
        motors(base*0.55, base*0.95)
    else:
        motors(base-steer, base+steer)
    time.sleep(0.07)
finally:
  stop(); log(event='WF_END')
