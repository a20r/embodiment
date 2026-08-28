import sys, time, json, random
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import motors, stop, turn_by, turn_to

LOG=open('/tmp/seek.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+"\n"); LOG.flush()

def d6():
    s=read_port("d6"); return float(s) if s else None

def sig(n=4):
    vs=[]
    for _ in range(n):
        v=d6()
        if v is not None: vs.append(v)
    return sum(vs)/len(vs) if vs else None

def most_open(L):
    best=None; bestv=-2
    for i,v in enumerate(L):
        vv=3.0 if v<0 else v
        if vv>bestv: bestv=vv; best=i
    return best

t_end=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 120)
last_ping=0
prev=sig()
falling=0
try:
  while time.time()<t_end:
    if time.time()-last_ping>2:
        write_port("d8","PING A"); last_ping=time.time()
    d9=read_port("d9")
    if d9 and 'goal=0' not in d9:
        log(event='GOAL', d9=d9); break
    L=lidar()
    if L is None: continue
    fl=[x for x in (L[0],L[1],L[15]) if x>0]
    front=min(fl) if fl else 9
    bump=read_port("d0")=='1'
    s=sig()
    log(c=compass(), s=s, front=round(front,2))
    if bump or front<0.25:
        motors(-60,-60); time.sleep(0.35); stop()
        L2=lidar() or L
        i=most_open(L2)
        turn_by(((22.5*i+180)%360)-180)
        prev=sig(); falling=0
        continue
    if prev is not None and s is not None:
        if s < prev - 0.015: falling+=1
        elif s > prev + 0.005: falling=0
    if falling>=3:
        stop()
        L2=lidar() or L
        cand=[i for i in range(16) if (L2[i]<0 or L2[i]>0.6)]
        i=random.choice(cand) if cand else most_open(L2)
        turn_by(((22.5*i+180)%360)-180)
        falling=0; prev=sig()
        continue
    prev = s if s is not None else prev
    motors(65,65)
    time.sleep(0.15)
finally:
  stop(); log(event='END')
