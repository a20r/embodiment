import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,('A-TO-B: READY. COMING. HOLD. beep%d'%i).encode()+b'\n'); os.close(fd)
        except Exception: pass
        i+=1; time.sleep(10)
threading.Thread(target=beacon,daemon=True).start()
def d11(n=5):
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
H=196.6   # known-rising heading
alt=0     # heading-trial offset index: +45,-45,+90,-90,+135,-135,180
altseq=[45,-45,90,-90,135,-135,180]
leg=0; flats=0
S('start',H=H)
t0=time.time()
while time.time()-t0<2700 and leg<40:
    # aim at H
    cur=float(rd('d4'))
    err=(H-cur+540)%360-180
    if abs(err)>6: turn(err)
    if rd('d5')=='1':
        print('EVENT d5=1',flush=True); S('D5'); break
    if rd('d0')=='1':
        print('EVENT d0=1',flush=True); S('GOAL'); break
    e0=S('leg%d-a'%leg)
    l=lidar(); head=min([x for x in (l[15],l[0],l[1]) if x>0]+[99])
    dur=180 if head>0.30 else 90
    d.set(1,1); time.sleep(dur); d.set(0,0); time.sleep(0.5)
    e1=S('leg%d-b'%leg,dur=dur)
    if e1['d5']=='1': print('EVENT d5=1',flush=True); S('D5'); break
    if e1['d0']=='1': print('EVENT d0=1',flush=True); S('GOAL'); break
    dd=e1['d11']-e0['d11']
    print('LEG%d dd=%+.3f H=%.0f'%(leg,dd,H),flush=True)
    if dd>0.004:
        alt=0; flats=0        # keep heading
    elif dd<-0.004:
        H=(H+altseq[min(alt,6)])%360; alt+=1; flats=0
        S('newH',H=round(H,1))
    else:
        flats+=1
        if flats>=2:
            H=(H+altseq[min(alt,6)])%360; alt+=1; flats=0
            S('newH-flat',H=round(H,1))
    leg+=1
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log,open('/memory/chase5.json','w'))
print('DONE',flush=True)
