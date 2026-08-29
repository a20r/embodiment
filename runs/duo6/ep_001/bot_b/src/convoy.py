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
    except: pass
    if score is not None:
        scores.append((now,score))
        while scores and now-scores[0][0]>15: scores.popleft()
    LOG.write(json.dumps({'t':round(now,2),'x':round(odo.x,3),'y':round(odo.y,3),
      'h':round(odo.h or -1,1),'l':l,'g':goal_flag,'d5':s5,'st':state})+'\n')
    if goal_flag!='0':
        if not goal_latched:
            goal_latched=True
            LOG.write(json.dumps({'ev':'GOAL','t':now})+'\n')
        stop()
        wr('d0','GOALFOUND! B: home in on me via range sensor. I am stopped at the goal. goal flag=1.')
        time.sleep(0.7)
        continue
    if now-last_tx>2.0:
        txn+=1
        if txn%2==0:
            wr('d0','A: NOGOAL yet. PLAN: I explore, you chase me via range signal. If range weak I wait. When my goal flag=1 I stop and say GOALFOUND; you come to me. ACK plan?')
        else:
            wr('d0','HELLO A d5=%s exploring'%(s5 or '?'))
        last_tx=now
    # tether throttle
    weak = score is not None and score<0.35
    verylow = score is not None and score<0.22
    if verylow and state=='follow':
        # backtrack uphill: u-turn
        if now-last_uturn>20:
            side=-side; last_uturn=now
        # slow crawl continues (wall follow will loop back region)
    if state=='follow':
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
            if weak: base=min(base,45)
            if verylow: base=min(base,25)
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
