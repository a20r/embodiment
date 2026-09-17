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
            os.write(fd, ('A-TO-B: A moving toward you slowly. STUCK NO GOAL NO. Please back up if blocking. beep%d'%i).encode()+b'\n')
            os.close(fd)
        except Exception: pass
        i+=1
        time.sleep(12)
threading.Thread(target=beacon,daemon=True).start()

def S(tag,**kw):
    e={'tag':tag,'t':round(time.time(),1),'d4':float(rd('d4')),'l':lidar(),
       'eL':int(float(rd('d9'))),'eR':int(float(rd('d6'))),'d11':float(rd('d11')),'d0':rd('d0'),'d5':rd('d5'),'d3':rd('d3')}
    e.update(kw); log.append(e)
    print('%-10s d4=%6.1f eL=%6d eR=%6d d11=%.2f d0=%s  %s'%(tag,e['d4'],e['eL'],e['eR'],e['d11'],e['d0'],' '.join('%4.1f'%x for x in e['l'])), flush=True)
    return e

S('start')
t0=time.time()
last_enc=(log[-1]['eL'],log[-1]['eR'])
while time.time()-t0 < 900:
    l=lidar()
    front=min(x for x in (l[15],l[0],l[1]) if x>0)
    left =sum(x for x in (l[13],l[14]) if x>0)/2
    right=sum(x for x in (l[3],l[4],l[5]) if x>0)/3
    err=left-right   # >0: left side has more room -> steer left? (left rays 13,14; right rays 3,4,5)
    if front>0.55:
        # cruise; gentle centering: steer toward side with more room
        if err>0.05:  d.set(0.85,1.0)     # more room on left -> turn left (right wheel faster)
        elif err<-0.05: d.set(1.0,0.85)
        else: d.set(1,1)
        mode='cruise'
    elif front>0.38:
        d.set(0.5,0.5); mode='creep'
    else:
        d.set(0,0); mode='blocked'
        # nudge protocol: transmit & wait, then gentle push
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(fd,b'A-TO-B: YOU ARE BLOCKING ME. BACK UP PLEASE.\n'); os.close(fd)
        except Exception: pass
        time.sleep(6)
        d.set(0.6,0.6); time.sleep(2.5); d.set(0,0); time.sleep(1.5)
        mode='pushed'
    S(mode,front=round(front,3),err=round(err,3))
    if rd('d0')=='1':
        print('GOAL FLAG d0=1 !!!',flush=True); S('GOAL?'); break
    time.sleep(3.5)
stop=True
d.set(0,0); time.sleep(0.3); d.close()
json.dump(log, open('/memory/nav2.json','w'))
print('DONE',flush=True)
