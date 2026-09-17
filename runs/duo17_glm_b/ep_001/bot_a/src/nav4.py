import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd, ('A-TO-B: A exiting slot now. GOAL NO STUCK NO. beep%d'%i).encode()+b'\n')
            os.close(fd)
        except Exception: pass
        i+=1; time.sleep(12)
threading.Thread(target=beacon,daemon=True).start()
def S(tag,**kw):
    e={'tag':tag,'t':round(time.time(),1),'d4':float(rd('d4')),'l':lidar(),
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d11':float(rd('d11')),'d0':rd('d0'),'d3':rd('d3')}
    e.update(kw); log.append(e)
    print('%-8s d4=%6.1f eL=%6d eR=%6d d11=%.2f d0=%s %s'%(tag,e['d4'],e['eL'],e['eR'],e['d11'],e['d0'],' '.join('%4.1f'%x for x in e['l'])), flush=True)
    return e
def opening(l):
    hot=[(i,v) for i,v in enumerate(l) if v>1.0]
    if not hot: return None
    sx=sum(v*math.sin(2*math.pi*i/16) for i,v in hot); cy=sum(v*math.cos(2*math.pi*i/16) for i,v in hot)
    if sx==0 and cy==0: return None
    return (math.atan2(sx,cy)/(2*math.pi)*16)%16
S('start')
t0=time.time(); n=0
while time.time()-t0<840 and n<300:
    n+=1
    l=lidar(); O=opening(l)
    head=min([x for x in (l[0],l[1],l[15]) if x>0]+[99])
    if O is None:
        d.set(1,1); time.sleep(5); d.set(0,0); time.sleep(0.3); S('blind%d'%n); continue
    E=((O+8)%16)-8
    if head<0.30:
        d.set(-1,-1); time.sleep(2.5); d.set(0,0); time.sleep(0.3)
        S('back%d'%n,E=round(E,2),head=round(head,2))
        continue
    if abs(E)>0.25:
        t=min(3.0,0.8*abs(E))
        d.set(1,-1) if E>0 else d.set(-1,1)
        time.sleep(t); d.set(0,0); time.sleep(0.3)
        S('turn%d'%n,E=round(E,2))
    else:
        d.set(1,1); time.sleep(6.0); d.set(0,0); time.sleep(0.3)
        S('fwd%d'%n,E=round(E,2),head=round(head,2))
    if rd('d0')=='1':
        S('GOALFLAG'); print('GOAL FLAG d0=1',flush=True); break
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log, open('/memory/nav4.json','w'))
print('DONE',flush=True)
