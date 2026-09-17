import json, time, math
from driver import Driver, rd, lidar

d = Driver()
log=[]
def snap(tag, extra=None):
    try:
        e={'tag':tag,'d4':float(rd('d4')),'l':lidar()}
        if extra: e.update(extra)
        log.append(e)
        print('%s d4=%6.1f  %s'%(tag,e['d4'],' '.join('%5.2f'%x for x in e['l'])), flush=True)
    except Exception as ex:
        print('snap err',ex, flush=True)

def doorway(l):
    # circular mean index of rays above threshold (16 rays, CW increasing)
    pts=[(i,v) for i,v in enumerate(l) if v>0.9]
    if not pts: return None
    sx=sum(v*math.sin(2*math.pi*i/16) for i,v in pts)
    cy=sum(v*math.cos(2*math.pi*i/16) for i,v in pts)
    ang=math.atan2(sx,cy)
    return (ang/(2*math.pi)*16)%16

def drive_test(secs=1.2):
    b=lidar(); d.set(1,1); time.sleep(secs); d.set(0,0); time.sleep(0.2)
    a=lidar()
    pairs=[(x,y) for x,y in zip(b,a) if x>0 and y>0]
    return max(abs(x-y) for x,y in pairs)

snap('start')
tries=0
aligned=0
while tries<40:
    tries+=1
    l=lidar()
    D=doorway(l)
    if D is None:
        d.set(-1,1); time.sleep(1.0); d.set(0,0); time.sleep(0.2)
        snap('scan%d'%tries); continue
    dist=((0-D+8)%16)-8
    snap('pos%d'%tries, {'D':D,'dist':dist})
    if abs(dist)<0.4:
        aligned+=1
        delta=drive_test()
        snap('drive%d'%tries,{'delta':delta})
        if delta>0.35:
            print('FREE! driving out',flush=True)
            d.set(1,1); time.sleep(4); d.set(0,0); time.sleep(0.3)
            snap('out')
            break
        time.sleep(0.2)
    else:
        aligned=0
        t=min(2.0, 0.45*abs(dist))
        if dist>0: d.set(-1,1)
        else: d.set(1,-1)
        time.sleep(t); d.set(0,0); time.sleep(0.2)
        snap('rot%d'%tries)
d.set(0,0); d.close()
json.dump(log, open('/memory/escape2.json','w'))
print('DONE tries=%d'%tries, flush=True)
