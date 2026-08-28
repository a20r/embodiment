import sys,time; sys.path.insert(0,'/memory/code')
from robot import Robot
r=Robot(); time.sleep(1)
def b0():
    s=r.scan(); return s[0] if s else None
print("start b0",b0(),flush=True)
for thr in [15,25,40]:
    r.cmd(0,thr); time.sleep(3); r.cmd(0,0); time.sleep(0.5)
    print(f"thr={thr} b0={b0()} bump={r.get(5)}",flush=True)
