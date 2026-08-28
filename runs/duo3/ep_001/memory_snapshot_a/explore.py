import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import motors, stop, turn_by

LOG = open('/tmp/explore.log','a')
def log(**kw):
    kw['t']=round(time.time(),1)
    LOG.write(json.dumps(kw)+"\n"); LOG.flush()

FWD=60
last_ping=0
t_end=time.time()+float(sys.argv[1]) if len(sys.argv)>1 else time.time()+60
try:
  while time.time()<t_end:
    now=time.time()
    if now-last_ping>2:
        write_port("d8","PING A")
        last_ping=now
    L=lidar(); c=compass()
    if L is None: continue
    d9=read_port("d9"); d0=read_port("d0"); d7=read_port("d7"); d6=read_port("d6")
    front=min(x for x in [L[0],L[1],L[15]] if True if x>0) if any(x>0 for x in [L[0],L[1],L[15]]) else 9
    fl=[x for x in (L[0],L[1],L[15]) if x>0]
    front=min(fl) if fl else 9
    left=L[4] if L[4]>0 else 9
    log(c=c, L=L, d9=d9, d0=d0, d7=d7, d6=d6)
    if d9 and 'goal=0' not in d9:
        log(event='GOAL', d9=d9); stop(); break
    if d0=='1' or front<0.22:
        # blocked: back a touch, turn right (compass negative)
        motors(-60,-60); time.sleep(0.4); stop()
        # choose more open side
        L2=lidar() or L
        leftd=L2[4] if L2[4]>0 else 9; rightd=L2[12] if L2[12]>0 else 9
        turn_by(80 if leftd>rightd else -80)
        continue
    # left wall follow: keep left ~0.3
    err = (left - 0.3)
    err = max(min(err, 0.4), -0.4)
    steer = err * 120   # positive err (too far) -> turn left (compass+): d5>d4
    if left>1.2:  # lost wall, arc left
        motors(FWD-25, FWD+25)
    else:
        motors(FWD - steer, FWD + steer)
    time.sleep(0.12)
finally:
  stop()
  log(event='END')
