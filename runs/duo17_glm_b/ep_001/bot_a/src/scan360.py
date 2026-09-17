import json, time, math
from driver import Driver, rd, lidar
d=Driver()
log=[]
d.set(-1,1)   # CCW
t0=time.time()
while time.time()-t0 < 170:
    try:
        log.append({'t':round(time.time()-t0,2),'d4':float(rd('d4')),'l':lidar(),'d6':rd('d6'),'d9':rd('d9'),'d11':rd('d11')})
    except Exception: pass
    time.sleep(0.18)
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log, open('/memory/scan360.json','w'))
print('done', len(log), flush=True)
