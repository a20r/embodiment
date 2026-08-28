import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import motors, stop, turn_by

LOG=open('/tmp/wf2.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+"\n"); LOG.flush()

def hot():
    d7=read_port("d7"); d9=read_port("d9")
    if d7 and d7.strip() not in ('0',''):
        log(event='D7',d7=d7); print("D7 HOT",d7,flush=True); return True
    if d9 and 'goal=' in d9 and 'goal=0' not in d9:
        log(event='GOALFLAG',d9=d9); print("GOALFLAG",d9,flush=True); return True
    return False

def rs():
    v=read_port("d6")
    return float(v) if v else None

t_end=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 900)
last_ping=0; last_log=0
try:
  while time.time()<t_end:
    if hot(): break
    now=time.time()
    s=rs()
    if now-last_ping>3:
        write_port("d8",f"B roaming; will HOLD when adjacent. s={s}")
        last_ping=now
    if s and s>1.15:
        stop()
        print("CLOSE! holding. s=",s,flush=True)
        held=time.time()
        while time.time()-held<30:
            if hot(): raise SystemExit
            write_port("d8",f"B ADJACENT holding s={rs()}. A: are you also holding? if both hold & no goal, we must co-travel: follow my signal, I will move SLOW to unexplored area")
            time.sleep(2)
            s2=rs()
            print("hold s=",s2,flush=True)
            if not s2 or s2<0.85: break
        continue
    L=lidar()
    if L is None: continue
    fl=[v for v in (L[0],L[1],L[15]) if v>0]
    front=min(fl) if fl else 9
    left=L[4] if L[4]>0 else 9
    lf=L[2] if L[2]>0 else 9
    bump=read_port("d0")=='1'
    if now-last_log>2:
        log(c=compass(), s=s, f=round(front,2)); last_log=now
    if bump:
        motors(-120,-120); time.sleep(0.35); stop(); turn_by(-80); continue
    if front<0.24:
        stop(); turn_by(-85); continue
    err=max(min(left-0.23,0.3),-0.3)
    steer=err*220
    if lf<0.2: steer-=35
    base=150 if front>0.6 else 90
    if left>0.8: motors(base*0.55, base*0.95)
    else: motors(base-steer, base+steer)
    time.sleep(0.07)
finally:
  stop(); log(event='WF2_END')
