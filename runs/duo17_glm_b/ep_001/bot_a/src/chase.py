import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,('A-TO-B: A IS MOVING, COMING TO YOU. STUCK NO GOAL NO. HOLD POSITION. beep%d'%i).encode()+b'\n')
            os.close(fd)
        except Exception: pass
        i+=1; time.sleep(10)
threading.Thread(target=beacon,daemon=True).start()
def d11(n=4):
    vals=[]
    for _ in range(n):
        try: vals.append(float(rd('d11')))
        except Exception: pass
        time.sleep(0.05)
    return sum(vals)/max(1,len(vals))
def S(tag,**kw):
    e={'tag':tag,'t':round(time.time(),1),'d4':float(rd('d4')),'d11':d11(),'l':lidar(),
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d0':rd('d0'),'d3':rd('d3')}
    e.update(kw); log.append(e)
    print('%-9s d4=%6.1f d11=%.3f eL=%6d eR=%6d d0=%s %s'%(tag,e['d4'],e['d11'],e['eL'],e['eR'],e['d0'],' '.join('%4.1f'%x for x in e['l'])),flush=True)
    return e
S('start')
t0=time.time(); n=0; leg=45.0
while time.time()-t0<1800 and n<60:
    n+=1
    e0=S('leg%d-start'%n)
    d_before=e0['d11']; h0=e0['d4']
    l=lidar(); head=min([x for x in (l[15],l[0],l[1]) if x>0]+[99])
    moved=False
    if head>0.32:
        d.set(1,1); time.sleep(leg); d.set(0,0); time.sleep(0.4); moved=True
        e1=S('leg%d-end'%n,head=round(head,2))
    else:
        d.set(0,0); time.sleep(0.5)
        # blocked: rotate toward open side
        l2=lidar()
        left=open_ = sum(x for x in l2[12:16] if x>0)
        right=sum(x for x in l2[2:6] if x>0)
        if left>right: d.set(-1,1)
        else: d.set(1,-1)
        time.sleep(2.2); d.set(0,0); time.sleep(0.4)
        e1=S('leg%d-avoid'%n)
        # try drive after avoid
        d.set(1,1); time.sleep(leg*0.6); d.set(0,0); time.sleep(0.4)
        e1=S('leg%d-end'%n)
    d_after=e1['d11']; h1=e1['d4']
    dd=d_after-d_before; dh=(h1-h0+540)%360-180
    e1['dd']=round(dd,4); e1['dh']=round(dh,1); e1['grad']=round(dd/max(0.01,abs(dh)/90),5)
    print('  LEG%d: dd11=%+.4f dheading=%+.1f'%(n,dd,dh),flush=True)
    # steering decision: if d11 fell, turn 90 deg CW; if rose, keep; if ~0, turn 45
    if dd < -0.006:
        d.set(1,-1); time.sleep(5.5); d.set(0,0); time.sleep(0.3); S('leg%d-turn90CW'%n)
    elif abs(dd)<=0.006:
        d.set(1,-1); time.sleep(2.7); d.set(0,0); time.sleep(0.3); S('leg%d-turn45CW'%n)
    if rd('d0')=='1':
        S('GOALFLAG'); print('GOAL FLAG!!',flush=True); break
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log,open('/memory/chase.json','w'))
print('DONE',flush=True)
