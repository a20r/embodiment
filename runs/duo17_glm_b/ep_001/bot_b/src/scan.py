import time, sys
sys.path.insert(0,'/bot/src')
from robot import *

# slow rotation, sample lidar vs heading
rows=[]
t0=time.time()
h0=heading()
motors(12,-12)
while time.time()-t0<40:
    h=heading()
    rows.append((round(h,1), lidar()))
    if len(rows)>=3 and ((h-h0)%360)<10 and time.time()-t0>15:
        break
    time.sleep(0.25)
stop()
print("start h", h0, "end h", heading(), "samples", len(rows))
import json
with open('/bot/src/scan1.json','w') as f: json.dump(rows,f)
# summarize: for each row print heading and the index of max reading
for h,l in rows[::3]:
    print(h, ['%.2f'%x for x in l])
