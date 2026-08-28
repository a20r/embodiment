import time, math, sys, json
from robolib import Bot, angdiff

b=Bot()
log=open('/memory/telemetry.log','a',buffering=1)
def L(*a):
    log.write(' '.join(str(x) for x in a)+'\n')

L('=== explore start', time.time())

# beams: 0=front, CCW, 22.5 deg each. right side = beam12, left = beam4, back=8
FRONT=[15,0,1]
def sensors():
    l=b.lidar()
    if l is None: return None
    # replace -1 dropouts with previous... just clamp to 3.0 (treat as unknown-far)? safer: keep small memory
    return l

prev=[0.5]*16
def clean(l):
    global prev
    out=[]
    for i,v in enumerate(l):
        if v<0: v=prev[i]
        out.append(v)
    prev=out
    return out

SPEED=60
last_tx=0
t0=time.time()
stuck_odo=b.odo() or 0
stuck_t=time.time()
state='follow'
while True:
    if b.goal():
        b.stop(); L('GOAL REACHED', time.time()); print('GOAL'); break
    l=sensors()
    if l is None:
        time.sleep(0.05); continue
    l=clean(l)
    front=min(l[15],l[0],l[1])
    fl=min(l[1],l[2])
    fr=min(l[14],l[13])
    right=l[12]; left=l[4]
    now=time.time()
    if now-last_tx>5:
        b.tx('hello'); last_tx=now
        rxm=b.rx.get()
        if rxm: L('RX:',rxm)
        L('t=%.1f'%(now-t0),'head',b.heading(),'odo',b.odo(),'lidar',','.join('%.2f'%x for x in l))
    # anti-stuck: if odo barely changes in 4s, back up and turn
    o=b.odo() or 0
    if abs(o-stuck_odo)>60:
        stuck_odo=o; stuck_t=now
    elif now-stuck_t>4:
        L('STUCK recover')
        b.wheels(-SPEED,-SPEED); time.sleep(0.8)
        b.wheels(40,-40); time.sleep(0.8)
        b.stop()
        stuck_odo=b.odo() or 0; stuck_t=time.time()
        continue
    # right-hand wall follow
    if front<0.28 or (front<0.4 and (fl<0.2 or fr<0.2)):
        # blocked: rotate left in place until front clear
        b.wheels(-40,40)
        time.sleep(0.15)
        continue
    # steering: keep right wall at ~0.3
    err=0.0
    if right<0.9:
        err=(0.3-right)   # too close -> positive -> steer left
    else:
        # right open: curve right to find wall
        err=-0.25
    # also avoid left wall
    if left<0.18: err+=(0.18-left)*2
    err=max(-0.5,min(0.5,err))
    turn=err*120
    sp=SPEED if front>0.6 else 35
    b.wheels(sp-turn, sp+turn)
    time.sleep(0.1)
