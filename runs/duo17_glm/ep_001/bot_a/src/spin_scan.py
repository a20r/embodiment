import sys, time
sys.path.insert(0, "/bot/src")
from robot import *

log = []
turn(30)  # deg/s clockwise
t0 = time.time()
h0 = heading()
while time.time() - t0 < 13:
    h = heading(); s = scan()
    if h is not None and s:
        log.append((round(time.time()-t0,2), h, s))
    time.sleep(0.15)
turn(0)
with open("/memory/scan1.txt","w") as f:
    for t,h,s in log:
        f.write(f"{t} {h} " + ",".join(f"{v:.2f}" for v in s) + "\n")
print("samples:", len(log), "h0:", h0, "h_end:", log[-1][1] if log else None)
