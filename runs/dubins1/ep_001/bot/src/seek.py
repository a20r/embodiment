import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
target=float(sys.argv[1]); dur=float(sys.argv[2])
def d6():
    return r.get(6)
res=n.turn_to(target, tol=15, timeout=75)
print("turn:",res,"h=",r.heading(),"d6=",d6(),flush=True)
t0=time.time()
while time.time()-t0<dur:
    if r.goal(): print("GOAL!!!",r.get(0),flush=True); break
    h=n.upd(); s=r.scan()
    f=n.clearance(0)
    if r.bump() or f<0.15:
        print("blocked f=",f,"d6=",d6(),flush=True)
        n.cmd(0,-12); time.sleep(0.8); n.stop()
        break
    steer=max(-90,min(90,3*angdiff(target,h))) if h is not None else 0
    n.cmd(steer, 14 if f>0.4 else 8)
    time.sleep(0.15)
    print(f"t={time.time()-t0:.1f} h={h:.0f} f={f:.2f} d6={d6()} d0={r.get(0)}",flush=True)
    time.sleep(0.5)
n.stop()
print("end scan",r.scan(),"h",r.heading(),"d6",d6(),flush=True)
