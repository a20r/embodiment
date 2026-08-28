import time, math, random
from robolib2 import Bot, angdiff

b=Bot()
log=open('/memory/telemetry.log','a',buffering=1)
def L(*a): log.write(' '.join(str(x) for x in a)+'\n')
L('=== ex4 start',time.time())

MT=0.00176
x,y=0.0,0.0
lasto=b.odo()
t0=time.time()
lastlog=0; lasttx=0; lastcell=0
hist=[]
cells={}
hand=+1
hand_until=0

def upd_pose(h):
    global x,y,lasto
    o=b.odo()
    if h is None: return
    d=(o-lasto)*MT; lasto=o
    x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))

def recover():
    L('RECOVER t=%.0f'%(time.time()-t0))
    b.wheels(-50,-50); time.sleep(0.9)
    if random.random()<0.5: b.wheels(-38,38)
    else: b.wheels(38,-38)
    time.sleep(random.choice([0.4,0.7,1.0]))
    b.stop()

try:
  while True:
    t,g=b.status()
    if g:
        b.stop(); L('GOAL! t=%.1f pos %.2f %.2f'%(time.time()-t0,x,y)); print('GOAL'); break
    l=b.lidar()
    h=b.heading()
    upd_pose(h)
    now=time.time()
    if now-lastcell>1.0:
        lastcell=now
        c=(round(x/0.6),round(y/0.6))
        cells[c]=cells.get(c,0)+1
        if cells[c]>6 and now>hand_until:
            hand=-hand
            hand_until=now+45
            cells={}
            L('SWITCH HAND to %d t=%.0f pos %.2f,%.2f'%(hand,now-t0,x,y))
    hist.append((now,list(l)))
    while hist and now-hist[0][0]>4.5: hist.pop(0)
    if now-t0>6 and len(hist)>8 and now-hist[0][0]>3.5:
        old=hist[0][1]
        if max(abs(a-c) for a,c in zip(l,old))<0.07:
            recover(); hist=[]; continue
    front=l[0]
    fr=min(l[14],l[15]); fl=min(l[1],l[2])
    right=min(l[12], l[11]*0.924, l[13]*0.924)
    left=min(l[4], l[3]*0.924, l[5]*0.924)
    if hand<0:
        left,right=right,left
        fl,fr=fr,fl
    if now-lastlog>1.0:
        L('t=%.0f pos %.2f,%.2f h=%.0f f=%.2f hand=%d odo=%d lid=%s'%(now-t0,x,y,h or -1,front,hand,lasto,','.join('%.2f'%v for v in l)))
        lastlog=now
    if now-lasttx>10:
        b.tx('hello'); lasttx=now
        m=b.rx()
        if m: L('RX:',m)
    def W(a,c):
        if hand<0: b.wheels(c,a)
        else: b.wheels(a,c)
    if fl<0.14 and front<0.5:
        W(30,-30); time.sleep(0.15); continue
    if fr<0.14 and front<0.5:
        W(-30,30); time.sleep(0.15); continue
    if front<0.32 or (front<0.45 and fr<0.22 and fl<0.22):
        W(-35,35); time.sleep(0.12); continue
    if right>0.55:
        W(48,20); time.sleep(0.12); continue
    err=(0.26-right)
    if left<0.18: err+=(0.18-left)*1.5
    err=max(-0.3,min(0.3,err))
    turn=err*140
    sp=50 if front>0.8 else 30
    W(sp-turn, sp+turn)
    time.sleep(0.08)
finally:
  b.stop()
