import time, math, random, statistics
from grid3 import Nav
from ctl import angdiff

b=Nav()
log=open('/memory/vhome.log','a',buffering=1)
t0=time.time()
def lg(m): log.write(f'{time.time()-t0:7.1f} {m}\n'); print(m,flush=True)

def sig(dur=1.6):
    b.p[6].poll(); b.p[6].queue.clear()
    t=time.time(); vals=[]
    while time.time()-t<dur:
        b.p[6].poll()
        vals+=[float(x) for x in b.p[6].queue]; b.p[6].queue.clear()
        time.sleep(0.04)
    return statistics.mean(vals) if vals else 0.0

def checks():
    b.p[9].poll()
    if b.p[9].last and 'goal=0' not in b.p[9].last: lg(f'GOALFLAG {b.p[9].last}')
    for m in b.radio_recv(): lg(f'RX {m}')

def leg(hdg,dur):
    """drive straight at heading for dur; return (blocked, dist_est)"""
    t=time.time(); blocked=False
    while time.time()-t<dur:
        ss=[b.sc(),b.sc(),b.sc()]
        s=[statistics.median(v) for v in zip(*ss)]
        h=b.heading()
        e=angdiff(hdg,h) if h is not None else 0
        w=max(min(1.6*e,40),-40)
        if s[0]<0.55 or min(s[1],s[15])<0.35:
            blocked=True; break
        if b.stalled(): blocked=True; break
        b.drive(0.55,w)
        time.sleep(0.05)
    b.stop()
    return blocked

hdg=b.heading() or 0
lg(f'VHOME start h={hdg:.0f}')
s0=sig()
fails=0
while True:
    checks()
    b.radio_send(f'A lost in void, homing on your signal s={s0:.3f}. Please keep beaconing / stay put if convenient.')
    blocked=leg(hdg,4.0)
    s1=sig()
    lg(f'leg h={hdg:.0f} s {s0:.3f}->{s1:.3f} blocked={blocked}')
    if blocked:
        lg('STRUCTURE/obstacle reached')
        if s1>0.12:
            lg('near maze/robot; stopping for next phase'); break
        # follow around: turn left, continue
        hdg=(hdg+90)%360
        s0=s1; continue
    if s1>0.55:
        lg('very close; stopping'); break
    if s1>s0*1.08+0.002:
        fails=0  # keep going
    else:
        fails+=1
        hdg=(hdg+random.choice([90,-90,135,-135,180]))%360
    s0=s1
