import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,('A-TO-B: A COMING. HOLD. beep%d'%i).encode()+b'\n'); os.close(fd)
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
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d0':rd('d0'),'d5':rd('d5')}
    e.update(kw); log.append(e)
    print('%s %s'%(tag,json.dumps({k:e[k] for k in ('d4','d11','eL','eR','d0','d5')})),flush=True)
    return e
def turn(deg):
    # deg>0 = CW ; 2.2 deg/s at (1,-1)
    t=abs(deg)/2.2
    d.set(1,-1) if deg>0 else d.set(-1,1)
    time.sleep(min(40,t)); d.set(0,0); time.sleep(0.3)
S('start')
t0=time.time(); n=0; hist=[]
W=254.7
while time.time()-t0<2400 and n<50:
    n+=1
    e0=S('leg%d-a'%n)
    if e0['d0']=='1' or e0['d5']=='1':
        print('EVENT d0=%s d5=%s - STOPPING'%(e0['d0'],e0['d5']),flush=True); break
    l=lidar(); head=min([x for x in (l[15],l[0],l[1]) if x>0]+[99])
    if head>0.30:
        d.set(1,1); time.sleep(80); d.set(0,0); time.sleep(0.4)
        e1=S('leg%d-b'%n,head=round(head,2))
    else:
        d.set(0,0); time.sleep(0.5)
        l2=lidar()
        if sum(x for x in l2[12:16] if x>0)>sum(x for x in l2[2:6] if x>0): turn(-35)
        else: turn(35)
        d.set(1,1); time.sleep(50); d.set(0,0); time.sleep(0.4)
        e1=S('leg%d-av'%n)
    if e1['d0']=='1' or e1['d5']=='1':
        print('EVENT d0=%s d5=%s - STOPPING'%(e1['d0'],e1['d5']),flush=True); break
    deL=e1['eL']-e0['eL']; deR=e1['eR']-e0['eR']
    dist=(deL+deR)/2.0
    dth=(deL-deR)/W  # CW-positive
    thm=math.radians(e0['d4'])+math.radians(dth)/2
    bear=math.degrees(thm)%360
    dd=e1['d11']-e0['d11']
    hist.append((bear,dist,dd))
    print('LEG%d dist=%.0fmm bear=%.0f dd11=%+.3f'%(n,dist,bear,dd),flush=True)
    # gradient estimate from history (last 8 legs)
    sx=sy=0.0; wsum=0.0
    for b,L,ddv in hist[-8:]:
        w=ddv/max(0.05,L)   # dd11 per meter
        r=math.radians(b)
        sx+=w*math.sin(r); sy+=w*math.cos(r); wsum+=abs(w)
    if wsum>0.02:
        grad=math.degrees(math.atan2(sx,sy))%360
        cur=e1['d4']
        err=(grad-cur+540)%360-180
        if abs(err)>25:
            turn(err)  # steer toward estimated good direction
            S('steer%d'%n,err=round(err,1),grad=round(grad,1))
        else:
            S('ok-hdg%d'%n,err=round(err,1))
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log,open('/memory/chase3.json','w'))
print('DONE',flush=True)
