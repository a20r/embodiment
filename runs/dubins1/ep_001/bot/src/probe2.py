import sys,time; sys.path.insert(0,'/memory/code')
from robot import Robot
r=Robot(); time.sleep(1)
def snap(tag):
    print(tag,"h",r.heading(),"bump",r.get(5),"scan",r.scan(),flush=True)
snap("start")
r.cmd(0,-10); time.sleep(3); r.cmd(0,0); time.sleep(0.5)
snap("after rev")
r.cmd(0,10); time.sleep(2); r.cmd(0,0); time.sleep(0.5)
snap("after fwd")
