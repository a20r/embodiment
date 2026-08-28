from probe import rd, wr
import time
def enc(): return (int(rd(2)), int(rd(8)))
for sp in [10,20,40,400]:
    # spin in place to avoid walls? no, drive fwd/back alternately: use rotation for calibration
    e0=enc(); t0=time.time()
    wr(6,str(sp)); wr(7,str(-sp))
    time.sleep(1.5)
    wr(6,"0"); wr(7,"0"); time.sleep(0.2)
    e1=enc(); dt=time.time()-t0
    print(sp, "counts/s", round((e1[0]-e0[0])/dt,1), round((e1[1]-e0[1])/dt,1))
