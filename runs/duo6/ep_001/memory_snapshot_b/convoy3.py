import sys, time, math, json, os, select, random
from collections import deque
sys.path.insert(0,'/bot/src')
from robot import rd, wr, lidar, enc, motors, stop, status

LOG=open('/memory/traj.jsonl','a',buffering=1)
RXLOG=open('/memory/radio_rx.log','a',buffering=1)

class PipeReader:
    def __init__(self,port,log=None):
        self.fd=os.open('/dev/robot/'+port,os.O_RDONLY|os.O_NONBLOCK)
        self.buf=b''; self.last=''; self.log=log
    def poll(self):
        while True:
            r,_,_=select.select([self.fd],[],[],0)
            if not r: break
            try: chunk=os.read(self.fd,4096)
            except BlockingIOError: break
            if not chunk: break
            self.buf+=chunk
            while b'\n' in self.buf:
                line,self.buf=self.buf.split(b'\n',1)
                s=line.decode().strip()
                if s:
                    self.last=s
                    if self.log: self.log.write('%f %s\n'%(time.time(),s))
        return self.last

p5=PipeReader('d5'); p4=PipeReader('d4',RXLOG)

def heading_fast():
    try: return float(rd('d1'))
    except: return None

class Odo:
    def __init__(self):
        self.x=0.0; self.y=0.0; self.h=None
        self.eL,self.eR=enc()
        self.UPC=0.0015650
    def update(self):
        eL,eR=enc()
        d=((eL-self.eL)+(eR-self.eR))/2*self.UPC
        self.eL,self.eR=eL,eR
        h=heading_fast()
        if h is not None:
            if self.h is None: self.h=h
            else:
                err=((h-self.h+180)%360)-180
                self.h=(self.h+0.3*err)%360
        hr=math.radians(self.h or 0)
        self.x+=d*math.cos(hr); self.y+=d*math.sin(hr)

odo=Odo()
try:
    import subprocess
    for ln in reversed(subprocess.run(['tail','-50','/memory/traj.jsonl'],capture_output=True,text=True).stdout.strip().split('\n')):
        try:
            d=json.loads(ln)
            if 'x' in d: odo.x=d['x']; odo.y=d['y']; break
        except: pass
except Exception: pass

TARGET=0.17
def clean(v): return 3.0 if v<0 else v

state='follow'; side=1
last_tx=0; txn=0
score=None
scores=deque()
last_uturn=time.time()
goal_latched=False
homing=False
turn_tgt=0
hist=deque()
last_hop=time.time()
hop_ex=hop_ey=0

while True:
  try:
    l=lidar()
    if l is None: time.sleep(0.05); continue
    L=[clean(v) for v in l]
    front=min(L[15],L[0],L[1])
    wleft=min(L[11],L[12],L[13]); wright=min(L[3],L[4],L[5])
    wall = wleft if side==1 else wright
    odo.update()
    st=status(); goal_flag=st.get('goal','0')
    now=time.time()
    s5=p5.poll(); rx=p4.poll()
    if rx and ('ATGOAL' in rx or rx.strip().startswith('GOALFOUND!')) and not homing:
        homing=True; state='run'
        LOG.write(json.dumps({'ev':'homing_start','rx':rx})+'\n')
    try: score=float(s5)
    except: pass
    if score is not None:
        scores.append((now,score))
        while scores and now-scores[0][0]>12: scores.popleft()
    LOG.write(json.dumps({'t':round(now,2),'x':round(odo.x,3),'y':round(odo.y,3),
      'h':round(odo.h or -1,1),'l':l,'g':goal_flag,'d5':s5,'st':state,'hm':homing})+'\n')
    if goal_flag!='0':
        if not goal_latched:
            goal_latched=True
            LOG.write(json.dumps({'ev':'GOAL','t':now})+'\n')
        stop()
        wr('d0','GOALFOUND! A stopped at goal, flag=1. B come to me via range sensor.')
        time.sleep(0.7)
        continue
    if now-last_tx>2.0:
        txn+=1
        if homing:
            wr('d0','A: homing to you via range, d5=%s'%(s5 or '?'))
        elif txn%2==0:
            wr('d0','A: plan active. exploring, you tail me. NOT at goal yet. use exact word ATGOAL only when your flag=1. d5=%s'%(s5 or '?'))
        else:
            wr('d0','HELLO A d5=%s exploring'%(s5 or '?'))
        last_tx=now
    if homing:
        # chemotaxis uphill toward B
        if state not in ('run','turn'): state='run'
        if state=='run' and score is not None and len(scores)>12 and now-last_uturn>10:
            old=scores[0][1]
            if score<old-0.02:
                turn_tgt=((odo.h or 0)+180)%360
                state='turn'; last_uturn=now
        if state=='run':
            if front<0.3:
                cands=[i for i in range(16) if L[i]>0.9 and not (5<=i<=11)]
                if not cands: cands=[i for i in range(16) if L[i]>0.6]
                if not cands: cands=[max(range(16),key=lambda k:L[k])]
                i=random.choice(cands)
                turn_tgt=((odo.h or 0)+22.5*i)%360
                state='turn'
            else:
                u=0.0
                u+= 18*max(0,0.25-min(L[13],L[14],L[15]))/0.25
                u-= 18*max(0,0.25-min(L[1],L[2],L[3]))/0.25
                base=100 if front>1.5 else (55 if front>0.7 else 32)
                if score is not None and score>0.93: base=min(base,35)
                motors(base+u, base-u)
        elif state=='turn':
            err=((turn_tgt-(odo.h or 0)+180)%360)-180
            if abs(err)<10: state='run'
            else:
                sp=max(min(err*2.2,55),-55)
                if 0<=sp<16: sp=16
                if -16<sp<0: sp=-16
                motors(sp,-sp)
        time.sleep(0.08); continue
    hist.append((now,odo.x,odo.y))
    while hist and now-hist[0][0]>100: hist.popleft()
    if state=='follow' and now-last_hop>100 and len(hist)>60:
        xs=[p[1] for p in hist]; ys=[p[2] for p in hist]
        if math.hypot(max(xs)-min(xs),max(ys)-min(ys))<8:
            i=max(range(16), key=lambda k: L[k])
            turn_tgt=((odo.h or 0)+22.5*i)%360
            hop_ex,hop_ey=odo.x,odo.y
            state='hopturn'; last_hop=now
            LOG.write(json.dumps({'ev':'hop'})+'\n')
    if state=='hopturn':
        err=((turn_tgt-(odo.h or 0)+180)%360)-180
        if abs(err)<8: state='hopdrive'
        else:
            sp=max(min(err*2,50),-50)
            if 0<=sp<15: sp=15
            if -15<sp<0: sp=-15
            motors(sp,-sp)
        time.sleep(0.08); continue
    if state=='hopdrive':
        if front<0.25 or math.hypot(odo.x-hop_ex,odo.y-hop_ey)>8:
            state='follow'; side=-side; last_hop=now
        else:
            u=0
            if L[2]<0.15: u=8
            if L[14]<0.15: u=-8
            motors(70-u,70+u)
        time.sleep(0.08); continue
    weak = score is not None and score<0.35
    verylow = score is not None and score<0.22
    if verylow and state=='follow' and now-last_uturn>20:
        side=-side; last_uturn=now
    # explored-mass centroid EMA (repel)
    try:
        cx=cx+(odo.x-cx)*0.0004; cy=cy+(odo.y-cy)*0.0004
    except NameError:
        cx,cy=odo.x,odo.y
    if state=='follow':
        if front<0.3:
            outb=math.degrees(math.atan2(odo.y-cy,odo.x-cx))%360
            best=None;bs=-1e9
            for i in range(16):
                if L[i]<0.7: continue
                ang=((odo.h or 0)+22.5*i)%360
                dd=abs(((ang-outb+180)%360)-180)
                sc=-dd/45.0+L[i]*0.6+random.random()*1.5
                if 5<=i<=11: sc-=1.2
                if sc>bs: bs=sc;best=i
            if best is None: best=max(range(16),key=lambda k:L[k])
            turn_tgt=((odo.h or 0)+22.5*best)%360
            state='turnj'
        else:
            u=0.0
            u+= 20*max(0,0.22-min(L[13],L[14],L[15]))/0.22
            u-= 20*max(0,0.22-min(L[1],L[2],L[3]))/0.22
            base=100 if front>1.5 else (60 if front>0.7 else 35)
            if weak: base=min(base,45)
            if verylow: base=min(base,25)
            motors(base+u, base-u)
    elif state=='turnj':
        err=((turn_tgt-(odo.h or 0)+180)%360)-180
        if abs(err)<10: state='follow'
        else:
            sp=max(min(err*2.2,55),-55)
            if 0<=sp<16: sp=16
            if -16<sp<0: sp=-16
            motors(sp,-sp)
    else:
        state='follow'
    time.sleep(0.08)
  except Exception as e:
    stop()
    LOG.write(json.dumps({'err':str(e)})+'\n')
    time.sleep(0.5)
