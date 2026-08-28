import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
r = Robot()
time.sleep(1)
def b(i):
    s=r.scan(); return s[i] if s else None
# reverse away from wall, timing beam0 growth at thr -10 vs -20
for thr in [-10,-20,-50]:
    b0=b(0); r.cmd(0,thr); time.sleep(2); r.stop(); time.sleep(0.5)
    b1=b(0)
    print(f"thr={thr}: beam0 {b0}->{b1} rate={(b1-b0)/2:.3f}/s", flush=True)
    r.cmd(0,10); time.sleep(2 if thr!=-50 else 2); r.stop(); time.sleep(0.5)
    print("  back fwd: beam0", b(0), flush=True)
