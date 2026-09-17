import time, sys, json
sys.path.insert(0,"/bot/src")
from robot import *
def spin(deg, rate=6):
    t = abs(deg)/(2.0*rate)
    if deg>0: motors(rate,-rate)
    else: motors(-rate,rate)
    time.sleep(t); motors(0,0); time.sleep(0.25)
best=None; rows=[]
for k in range(16):
    time.sleep(0.3)
    ls=[lidar() for _ in range(3)]
    ls=[x for x in ls if x]
    b8=sum(x[8] for x in ls)/len(ls)
    h=heading()
    rows.append((k,round(h,1),round(b8,3)))
    print(k, round(h,1), round(b8,3), flush=True)
    if 0.55<b8<1.25 and (best is None or b8<best[1]): best=(k,round(b8,3),round(h,1))
    spin(22.5)
json.dump({"best":best,"rows":rows}, open("/bot/src/wallpick.json","w"))
print("BEST",best, flush=True)
