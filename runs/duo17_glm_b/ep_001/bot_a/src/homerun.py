import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def beacon():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,('A-TO-B: A COMING TO YOU. HOMING ON YOUR SIGNAL. beep%d'%i).encode()+b'\n')
            os.close(fd)
        except Exception: pass
        i+=1; time.sleep(12)
threading.Thread(target=beacon,daemon=True).start()
def d11(n=3):
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
    print('%-8s d4=%6.1f d11=%.3f eL=%6d eR=%6d d0=%s %s'%(tag,e['d4'],e['d11'],e['eL'],e['eR'],e['d0'],' '.join('%4.1f'%x for x in e['l'])),flush=True)
    return e
S('start')
t0=time.time(); n=0
best_h=None; best_d=-1
while time.time()-t0<1500 and n<400:
    n+=1
    e0=S('probe%d'%n)
    d0=e0['d11']
    # hill-climb heading: try +15deg (CCW ~7s at 2.2deg/s) and -15deg
    def trial(sign):
        d.set(-1,1) if sign>0 else d.set(1,-1)
        time.sleep(6.5); d.set(0,0); time.sleep(0.4)
        return d11()
    dp=trial(+1); h_after_p=float(rd('d4'))
    e1=S('tryCCW',d11after=round(dp,3))
    if dp> d0+0.004:
        continue  # keep new heading, loop again (maybe more CCW helps)
    # else try CW from original-ish
    dm=trial(-1); trial(-1) if False else None
    e2=S('tryCW',d11after=round(dm,3))
    if dm<=dp-0.004 and dm< d0-0.004:
        # both worse than start: go back CCW once
        trial(+1); S('back',d11after=round(d11(),3))
    if rd('d0')=='1':
        S('GOALFLAG'); print('GOAL FLAG!!',flush=True); break
    # drive forward between probes
    l=lidar(); head=min([x for x in (l[15],l[0],l[1]) if x>0]+[99])
    if head>0.32:
        d.set(1,1); time.sleep(25.0); d.set(0,0); time.sleep(0.4)
        S('drive%d'%n,head=round(head,2))
    else:
        d.set(0,0); time.sleep(1)
        S('blocked%d'%n,head=round(head,2))
    if rd('d0')=='1':
        S('GOALFLAG'); print('GOAL FLAG!!',flush=True); break
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log,open('/memory/homerun.json','w'))
print('DONE',flush=True)
