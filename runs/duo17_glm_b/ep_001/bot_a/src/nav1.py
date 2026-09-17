import json, time, math
from driver import Driver, rd, lidar
d=Driver()
log=[]
def snap(tag):
    e={'tag':tag,'t':round(time.time(),1),'d4':float(rd('d4')),'l':lidar(),'d6':int(float(rd('d6'))),'d9':int(float(rd('d9'))),'d11':float(rd('d11')),'d0':rd('d0'),'d5':rd('d5')}
    log.append(e); print(tag,'d4=%6.1f d6=%d d9=%d d11=%.3f d0=%s d5=%s'%(e['d4'],e['d6'],e['d9'],e['d11'],e['d0'],e['d5']), ' '.join('%5.2f'%x for x in e['l']), flush=True)
    return e
def doorway(l):
    hot=[(i,v) for i,v in enumerate(l) if v>1.2]
    if not hot: return None
    sx=sum(v*math.sin(2*math.pi*i/16) for i,v in hot); cy=sum(v*math.cos(2*math.pi*i/16) for i,v in hot)
    return (math.atan2(sx,cy)/(2*math.pi)*16)%16
snap('start')
# rotate CCW until doorway near ray0 (wrap zone 15.5..0.5)
for k in range(40):
    D=doorway(lidar())
    if D is not None and (D>15.4 or D<0.6):
        break
    d.set(-1,1); time.sleep(1.0); d.set(0,0); time.sleep(0.15)
snap('aligned D=%s'%D)
# long drive toward doorway
d.set(1,1)
for k in range(20):
    time.sleep(3.0)
    l=lidar(); D=doorway(l)
    e=snap('drive%02d'%k)
    log[-1]['D']=D
    # keep pushing; slight steering toward doorway
    if D is not None:
        if D>0.8 and D<8: d.set(1,0.75)     # doorway drifting CW-low: steer
        elif D<15.2: d.set(0.75,1)
        else: d.set(1,1)
d.set(0,0); time.sleep(0.3)
snap('end')
d.close()
json.dump(log, open('/memory/nav1.json','w'))
print('DONE',flush=True)
