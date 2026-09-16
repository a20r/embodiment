import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc(): 
    a=rd("d6"); b=rd("d9")
    return (float(a) if a is not None else None, float(b) if b is not None else None)
def test(l,r,t):
    h0=heading(); e0=enc(); motors(l,r); time.sleep(t); motors(0,0); time.sleep(0.4)
    e1=enc(); h1=heading()
    print("cmd(%s,%s) %.1fs: d6=%s d9=%s (d6 %+0.0f, d9 %+0.0f), h %.1f->%.1f (%+.1f)" % (l,r,t,e1[0],e1[1],e1[0]-e0[0],e1[1]-e0[1],h0,h1,h1-h0))
test(0,0,1)
test(2,-2,4)
test(-2,2,4)
test(2,2,4)
test(0,2,4)
test(2,0,4)
