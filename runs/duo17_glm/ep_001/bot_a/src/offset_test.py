import sys,time,math
sys.path.insert(0,"/bot/src")
from robot import *
from robust import rline
def h(): 
    v=rline("d4"); return float(v) if v else None
def sc():
    v=rline("d2")
    try: return [float(x) for x in v.split(",")]
    except: return None
# turn to heading ~85
start=h(); target=85.0
diff=(target-start+540)%360-180
turn(30 if diff>0 else -30)
t0=time.time()
while time.time()-t0<abs(diff)/30+1:
    c=h()
    e=(target-c+540)%360-180
    if abs(e)<3: break
    time.sleep(0.05)
turn(0)
time.sleep(0.5)
print("heading now:", h())
print("scan:", sc())
speed(3)
time.sleep(2)
speed(0); time.sleep(0.3)
print("after fwd: heading", h())
print("scan:", sc())
print("d5:", rline("d5"), "d6:", rline("d6"), "d9:", rline("d9"))
