import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
r = Robot()
time.sleep(1)
def hdg():
    vals=[]
    for _ in range(5):
        h=r.heading()
        if h is not None: vals.append(h)
        time.sleep(0.25)
    x=sum(math.cos(math.radians(v)) for v in vals); y=sum(math.sin(math.radians(v)) for v in vals)
    return round(math.degrees(math.atan2(y,x))%360,1)
def beam0():
    s=r.scan(); return s[0] if s else None
for steer,thr,dur in [(90,0.5,4),(90,1,4),(90,-1,4),(90,0.2,4)]:
    h0=hdg(); b0=beam0()
    r.cmd(steer,thr); time.sleep(dur); r.stop(); time.sleep(0.5)
    h1=hdg(); b1=beam0()
    dh=((h1-h0+180)%360)-180
    print(f"steer={steer} thr={thr}: dh={dh:.1f} ({dh/dur:.2f} deg/s) beam0 {b0}->{b1}", flush=True)
