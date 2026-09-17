import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,('A-TO-B: A MOVING TO YOU. HOLD. beep%d'%i).encode()+b'\n'); os.close(fd)
        except Exception: pass
        i+=1; time.sleep(8)
threading.Thread(target=beacon,daemon=True).start()
def d11(n=5):
    v=[]
    for _ in range(n):
        try: v.append(float(rd('d11')))
        except Exception: pass
        time.sleep(0.04)
    return sum(v)/max(1,len(v))
def S(tag,**kw):
    e={'tag':tag,'t':round(time.time(),1),'d4':round(float(rd('d4')),1),'d11':round(d11(),3),
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d0':rd('d0')}
    e.update(kw); log.append(e)
    print('%s %s'%(tag,json.dumps({k:e[k] for k in ('d4','d11','eL','eR','d0')})),flush=True)
    return e
S('start')
t0=time.time(); n=0; falls=0; ema=None
while time.time()-t0<2300 and n<40:
    n+=1
    e0=S('leg%d-a'%n)
    if e0['d0']=='1': print('GOAL!!',flush=True); break
    l=lidar(); head=min([x for x in (l[15],l[0],l[1]) if x>0]+[99])
    if head>0.30:
        d.set(1,1); time.sleep(80); d.set(0,0); time.sleep(0.4)
        e1=S('leg%d-b'%n,head=round(head,2))
    else:
        d.set(0,0); time.sleep(0.5)
        l2=lidar()
        if sum(x for x in l2[12:16] if x>0)>sum(x for x in l2[2:6] if x>0): d.set(-1,1)
        else: d.set(1,-1)
        time.sleep(2.2); d.set(0,0); time.sleep(0.3)
        d.set(1,1); time.sleep(50); d.set(0,0); time.sleep(0.4)
        e1=S('leg%d-av'%n)
    if e1['d0']=='1': print('GOAL!!',flush=True); break
    dd=e1['d11']-e0['d11']
    ema = dd if ema is None else 0.5*ema+0.5*dd
    print('LEG%d dd=%+.3f ema=%+.3f'%(n,dd,ema),flush=True)
    if ema<-0.015:
        falls+=1
        if falls>=1:
            d.set(1,-1); time.sleep(5.6); d.set(0,0); time.sleep(0.3); S('turn90'); falls=0; ema=None
    elif ema>0.015:
        falls=0  # keep heading
    else:
        d.set(1,-1); time.sleep(2.8); d.set(0,0); time.sleep(0.3); S('turn45'); ema=None
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log,open('/memory/chase2.json','w'))
print('DONE',flush=True)
