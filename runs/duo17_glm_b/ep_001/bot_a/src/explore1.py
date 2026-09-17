import json, time
from driver import Driver, rd, lidar
d = Driver()
log=[]
def snap(tag):
    try:
        log.append({'tag':tag,'t':time.time(),'d4':float(rd('d4')),'l':lidar()})
    except Exception as e:
        log.append({'tag':tag,'err':str(e)})
snap('start')
for i in range(14):
    d.set(1,-1); time.sleep(2.0); d.set(0,0); time.sleep(0.3)
    snap('rot%d'%i)
    before=lidar()
    d.set(1,1); time.sleep(1.5); d.set(0,0); time.sleep(0.3)
    after=lidar()
    delta=max(abs(a-b) for a,b in zip(before,after) if a>0 and b>0)
    snap('drive%d'%i)
    log[-1]['delta']=delta
    print('iter %d drive delta=%.2f d4=%.1f'%(i,delta,log[-1]['d4']), flush=True)
    if delta>0.35:
        print('MOTION at iter',i,flush=True)
        break
d.close()
json.dump(log, open('/memory/explore1.json','w'), indent=0)
for e in log:
    if 'l' in e:
        print('%s d4=%6.1f'%(e['tag'],e['d4']), ' '.join('%5.2f'%x for x in e['l']))
