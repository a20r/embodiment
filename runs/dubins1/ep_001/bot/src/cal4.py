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
h0=hdg()
print("h0",h0, "scan", r.scan(), flush=True)
r.cmd(90,8)
for i in range(8):
    time.sleep(1)
    print(i, "h=",r.heading(), "bump=",r.get(5), "b0=",(r.scan() or [None])[0], flush=True)
r.stop(); time.sleep(0.5)
print("h1",hdg(), "scan", r.scan(), flush=True)
