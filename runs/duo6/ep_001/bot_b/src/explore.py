import sys, time, math, json, os, threading, select
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

p5=PipeReader('d5')
p4=PipeReader('d4',RXLOG)

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
            if 'x' in d:
                odo.x=d['x']; odo.y=d['y']; break
        except: pass
except Exception: pass

TARGET=0.17
def clean(v): return 3.0 if v<0 else v

state='follow'
side=1
last_tx=0
hop_tgt=0; hop_ex=0; hop_ey=0
last_hop=time.time()
hist=deque()
scores=deque()   # (t,score)
best=0.0
last_uturn=0

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
    s5=p5.poll(); p4.poll()
    try: score=float(s5)
    except: score=None
    if score is not None:
        scores.append((now,score))
        while scores and now-scores[0][0]>20: scores.popleft()
        if score>best: best=score
    LOG.write(json.dumps({'t':round(now,2),'x':round(odo.x,3),'y':round(odo.y,3),
      'h':round(odo.h or -1,1),'l':l,'g':goal_flag,'d5':s5,'st':state})+'\n')
    if goal_flag!='0':
        stop()
        wr('d0','GOAL REACHED x=%.2f y=%.2f'%(odo.x,odo.y))
        time.sleep(1)
        continue
    if now-last_tx>1.0:
        wr('d0','HELLO A t=%d x=%.2f y=%.2f d5=%s'%(now,odo.x,odo.y,s5))
        last_tx=now
    hist.append((now,odo.x,odo.y))
    while hist and now-hist[0][0]>90: hist.popleft()
    # gradient u-turn: if score dropped noticeably over window, reverse
    if state=='follow' and score is not None and len(scores)>20 and now-last_uturn>25:
        old=scores[0][1]
        if score < old-0.02 and score < best-0.03:
            side=-side; last_uturn=now
            state='uturn'; hop_tgt=((odo.h or 0)+180)%360
            LOG.write(json.dumps({'ev':'uturn','score':score,'old':old,'side':side})+'\n')
    if state=='follow' and now-last_hop>90 and len(hist)>50:
        xs=[p[1] for p in hist]; ys=[p[2] for p in hist]
        if math.hypot(max(xs)-min(xs),max(ys)-min(ys))<10:
            i=max(range(16), key=lambda k: L[k])
            hop_tgt=((odo.h or 0)+22.5*i)%360
            hop_ex,hop_ey=odo.x,odo.y
            state='hopturn'; last_hop=now
            LOG.write(json.dumps({'ev':'hop','tgt':hop_tgt})+'\n')
    if state in ('hopturn','uturn'):
        err=((hop_tgt-(odo.h or 0)+180)%360)-180
        if abs(err)<8:
            state='hopdrive' if state=='hopturn' else 'follow'
        else:
            sp=max(min(err*2,50),-50)
            if 0<=sp<15: sp=15
            if -15<sp<0: sp=-15
            motors(sp,-sp)
    elif state=='hopdrive':
        if front<0.25 or math.hypot(odo.x-hop_ex,odo.y-hop_ey)>8:
            state='follow'; side=-side; last_hop=now
        else:
            u=0
            if L[2]<0.15: u=8
            if L[14]<0.15: u=-8
            motors(70-u,70+u)
    elif state=='follow':
        if front<0.14:
            state='pivot'
            motors(40*side,-40*side)
        else:
            u=25*(wall-TARGET)
            if wall>0.55: u=30
            u=max(-35,min(30,u))
            if front>1.2: base=90
            elif front>0.5: base=55
            else: base=28
            if wall>0.55: base=min(base,50)
            u*=side
            motors(base-u, base+u)
    elif state=='pivot':
        if front>0.35:
            state='follow'
        else:
            motors(40*side,-40*side)
    time.sleep(0.08)
  except Exception as e:
    stop()
    LOG.write(json.dumps({'err':str(e)})+'\n')
    time.sleep(0.5)
