import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,('A-TO-B: READY. HOMING ON YOU. beep%d'%i).encode()+b'\n'); os.close(fd)
        except Exception: pass
        i+=1; time.sleep(10)
threading.Thread(target=beacon,daemon=True).start()
def d11(n=4):
    v=[]
    for _ in range(n):
        try: v.append(float(rd('d11')))
        except Exception: pass
        time.sleep(0.05)
    return sum(v)/max(1,len(v))
def S(tag,**kw):
    e={'tag':tag,'t':round(time.time(),1),'d4':round(float(rd('d4')),1),'d11':round(d11(),3),
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d0':rd('d0'),'d5':rd('d5')}
    e.update(kw); log.append(e)
    print('%s %s'%(tag,json.dumps({k:e[k] for k in ('d4','d11','eL','eR','d0','d5')})),flush=True)
    return e
def turn(deg,rate=2.2):
    t=abs(deg)/rate
    d.set(1,-1) if deg>0 else d.set(-1,1)
    time.sleep(t); d.set(0,0); time.sleep(0.4)
def scan():
    # rotate 360 in 22.5deg steps, record d11
    best=(-1,None); prof=[]
    for k in range(16):
        time.sleep(1.2)
        v=d11()
        h=float(rd('d4'))
        prof.append((round(h,1),round(v,3)))
        if v>best[0]: best=(v,h)
        turn(22.5)
    S('scan',prof=prof,best_d11=best[0],best_h=best[1])
    return best
S('start')
best=scan()
# aim at best heading then drive 1m legs
for leg in range(12):
    cur=float(rd('d4'))
    err=(best[1]-cur+540)%360-180
    if abs(err)>8: turn(err)
    # check events before driving
    if rd('d5')=='1':
        print('EVENT d5=1 - B IN SIGHT - STOP',flush=True); S('D5EVENT'); break
    if rd('d0')=='1':
        print('EVENT d0=1 - AT GOAL',flush=True); S('GOAL'); break
    e0=S('leg%d-a'%leg)
    d.set(1,1); time.sleep(195); d.set(0,0); time.sleep(0.5)
    e1=S('leg%d-b'%leg)
    if e1['d5']=='1' or rd('d5')=='1':
        print('EVENT d5=1 - STOP',flush=True); S('D5EVENT'); break
    if e1['d0']=='1' or rd('d0')=='1':
        print('EVENT d0=1 - AT GOAL',flush=True); S('GOAL'); break
    if e1['d11']>0.95:
        print('d11>0.95 - STOP',flush=True); S('D11HIGH'); break
    best=scan()
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log,open('/memory/chase4.json','w'))
print('DONE',flush=True)
