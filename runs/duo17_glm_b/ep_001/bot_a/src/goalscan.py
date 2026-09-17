import json, time, math, collections
from driver import Driver, rd, lidar
d=Driver()
log=[]
d.set(-1,1)  # CCW spin
t0=time.time()
while time.time()-t0<200:
    try:
        e={'t':round(time.time()-t0,1),'d4':float(rd('d4')),'d11':float(rd('d11')),'l':lidar()}
        log.append(e)
    except Exception: pass
    time.sleep(0.5)
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log,open('/memory/goalscan.json','w'))
bins=collections.defaultdict(list)
for e in log:
    bins[round(e['d4']/10)*10].append(e['d11'])
print('heading-bin: mean d11 (n)')
for k in sorted(bins):
    print('%5d: %.3f (n=%d)'%(k%360, sum(bins[k])/len(bins[k]), len(bins[k])))
print('DONE',flush=True)
