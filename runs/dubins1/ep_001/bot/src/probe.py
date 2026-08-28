import sys,time; sys.path.insert(0,'/memory/code')
from robot import Robot
r=Robot(); time.sleep(1)
# creep forward slowly, print scan each 0.5s until bump, then stop
r.cmd(0,6)
for i in range(30):
    time.sleep(0.5)
    s=r.scan(); b=r.get(5); h=r.heading()
    print(i, "h",h,"bump",b,"s0..3",s[:4] if s else None, flush=True)
    if b=='1':
        print("BUMP scan:",s, flush=True); break
r.cmd(0,0)
