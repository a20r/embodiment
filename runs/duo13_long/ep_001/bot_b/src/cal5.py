import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc():
    a=rd("d6"); b=rd("d9")
    return (float(a) if a is not None else 0.0, float(b) if b is not None else 0.0)
def run(l,r,t,tag):
    print("== %s cmd(%s,%s)" % (tag,l,r))
    motors(0,0); time.sleep(0.6)
    e0=enc(); h0=heading(); seq=[]
    motors(l,r)
    t0=time.time()
    while time.time()-t0<t:
        time.sleep(0.5); seq.append(enc())
    motors(0,0); time.sleep(0.5)
    e1=enc()
    print("  d6 per 0.5s:", [round(s[0]-e0[0],1) for s in seq])
    print("  d9 per 0.5s:", [round(s[1]-e0[1],1) for s in seq])
    print("  h %.1f -> %.1f (%+.1f)" % (h0, heading(), heading()-h0))
run(2,0,4,"A")
run(0,2,4,"B")
run(2,2,4,"C")
run(2,-2,4,"D")
run(-2,2,4,"E")
