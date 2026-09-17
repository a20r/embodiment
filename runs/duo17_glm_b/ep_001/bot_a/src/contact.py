import json, time, math, threading, os
from driver import Driver, rd, lidar
d=Driver()
log=[]; stop=False
def spam():
    i=0
    while not stop:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,('A-TO-B: READY I SEE YOU. beep%d'%i).encode()+b'\n'); os.close(fd)
        except Exception: pass
        i+=1; time.sleep(1.5)
threading.Thread(target=spam,daemon=True).start()
def d11(n=4):
    v=[]
    for _ in range(n):
        try: v.append(float(rd('d11')))
        except Exception: pass
        time.sleep(0.04)
    return sum(v)/max(1,len(v))
def S(tag,**kw):
    e={'tag':tag,'t':round(time.time(),1),'d4':round(float(rd('d4')),1),'d11':round(d11(),3),
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d0':rd('d0'),'d5':rd('d5'),'d3':rd('d3'),
       'l':[round(x,2) for x in lidar()]}
    e.update(kw); log.append(e)
    print('%s %s'%(tag,json.dumps(e)),flush=True)
    return e
S('contact-start')
t0=time.time()
while time.time()-t0<1200:
    e=S('tick')
    if e['d0']=='1':
        print('*** AT GOAL - HOLDING ***',flush=True); break
    l=e['l']
    # find candidate B ray: ray with 0.25-0.9m reading that has deeper readings beside it
    cand=[i for i,v in enumerate(l) if 0.22<v<0.95]
    if e['d5']=='1' or e['d11']>0.93:
        # creep toward the closest candidate ray
        if cand:
            # aim: pick ray nearest 0.35m reading
            i=min(cand,key=lambda i:abs(l[i]-0.35))
            E=((i+8)%16)-8
            if abs(E)>0.4:
                turn_t=min(4.0,abs(E)*0.45)
                d.set(1,-1) if E>0 else d.set(-1,1)
                time.sleep(turn_t); d.set(0,0); time.sleep(0.3)
                S('aim',ray=i)
            d.set(1,1); time.sleep(6); d.set(0,0); time.sleep(0.3)
            S('creep',ray=i)
        else:
            d.set(0,0); time.sleep(1.0); S('wait')
    else:
        # lost it: hold
        d.set(0,0); time.sleep(1.5); S('hold')
stop=True
d.set(0,0); d.close()
json.dump(log,open('/memory/contact.json','w'))
print('DONE',flush=True)
