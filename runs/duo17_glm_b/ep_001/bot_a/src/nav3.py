import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]
stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd, ('A-TO-B: A moving out of slot toward you. STUCK NO GOAL NO. If you block me please reverse 2m. beep%d'%i).encode()+b'\n')
            os.close(fd)
        except Exception: pass
        i+=1; time.sleep(12)
threading.Thread(target=beacon,daemon=True).start()
def S(tag,**kw):
    e={'tag':tag,'t':round(time.time(),1),'d4':float(rd('d4')),'l':lidar(),
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d11':float(rd('d11')),'d0':rd('d0'),'d5':rd('d5'),'d3':rd('d3')}
    e.update(kw); log.append(e)
    print('%-8s d4=%6.1f eL=%6d eR=%6d d11=%.2f d0=%s %s'%(tag,e['d4'],e['eL'],e['eR'],e['d11'],e['d0'],' '.join('%4.1f'%x for x in e['l'])), flush=True)
    return e
def opening(l):
    hot=[(i,v) for i,v in enumerate(l) if v>1.0]
    if not hot: return None
    sx=sum(v*math.sin(2*math.pi*i/16) for i,v in hot); cy=sum(v*math.cos(2*math.pi*i/16) for i,v in hot)
    a=math.atan2(sx,cy)/(2*math.pi)*16
    return a%16
S('start')
t0=time.time(); pulses=0
while time.time()-t0<900 and pulses<400:
    l=lidar(); O=opening(l)
    front=min([x for x in (l[15],l[0],l[1],l[2]) if x>0]+[99])
    if O is None:
        d.set(1,1); time.sleep(4); d.set(0,0); time.sleep(0.4); pulses+=1
        S('blind%d'%pulses); continue
    E=((O-0+8)%16)-8   # signed rays from forward; + = opening CW of forward
    if abs(E)<0.35 and front>0.35:
        d.set(1,1); time.sleep(5.0); d.set(0,0); time.sleep(0.3); pulses+=1
        S('fwd%d'%pulses,E=round(E,2))
    elif front<=0.33:
        d.set(0,0)
        # blocked: turn toward wider side
        wide = 'L' if sum(l[13:16])>sum(l[3:6]) else 'R'
        d.set(-1,1) if wide=='L' else d.set(1,-1)
        time.sleep(1.6); d.set(0,0); time.sleep(0.3); pulses+=1
        S('avoid%d'%pulses,wide=wide,E=round(E,2))
    else:
        # pivot toward opening: opening CW (E>0) -> rotate CW: (1,-1)
        t=min(3.0,0.8*abs(E))
        if E>0: d.set(1,-1)
        else: d.set(-1,1)
        time.sleep(t); d.set(0,0); time.sleep(0.3); pulses+=1
        S('turn%d'%pulses,E=round(E,2))
    if rd('d0')=='1':
        S('GOALFLAG'); print('GOAL FLAG!',flush=True); break
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log, open('/memory/nav3.json','w'))
print('DONE',flush=True)
