import json, time
from driver import Driver, rd, lidar
d = Driver()
log=[]
def snap(tag):
    try: log.append({'tag':tag,'d4':float(rd('d4')),'l':lidar()})
    except Exception as e: log.append({'tag':tag,'err':str(e)})
def rot(sign, secs):
    d.set(sign*1.0, -sign*1.0); time.sleep(secs); d.set(0,0); time.sleep(0.2)
def drive_test(secs=1.5):
    before=lidar()
    d.set(1,1); time.sleep(secs); d.set(0,0); time.sleep(0.2)
    after=lidar()
    pairs=[(a,b) for a,b in zip(before,after) if a>0 and b>0]
    return max(abs(a-b) for a,b in pairs)
best=None
snap('start')
for i in range(60):
    delta = drive_test()
    snap('drive%d'%i); log[-1]['delta']=delta
    print('drive %d delta=%.2f d4=%.1f'%(i,delta,log[-1]['d4']), flush=True)
    if delta>0.4:
        print('MOVING! iter',i,flush=True)
        # keep driving
        d.set(1,1); time.sleep(3); d.set(0,0); time.sleep(0.3); snap('drove3s')
        break
    rot(+1, 1.0)   # small clockwise-nudge rotation between tests
    snap('rot%d'%i)
d.close()
json.dump(log, open('/memory/escape.json','w'))
print('DONE', flush=True)
