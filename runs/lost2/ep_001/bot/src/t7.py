from probe import rd, wr
import time
def scan(): return [float(x) for x in rd(1).split(",")]
def enc(): return (int(rd(2)), int(rd(8)))
e0=enc(); t0=time.time()
wr(6,"100"); wr(7,"100")
time.sleep(2)
wr(6,"0"); wr(7,"0"); time.sleep(0.3)
e1=enc(); dt=time.time()-t0
print("speed100: counts/s", (e1[0]-e0[0])/dt, (e1[1]-e0[1])/dt)
print(scan(), rd(5), rd(0))
