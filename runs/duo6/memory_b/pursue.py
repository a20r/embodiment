import sys, time, math, json, os, select, random
from collections import deque
sys.path.insert(0,'/bot/src')
from robot import rd, wr, lidar, enc, motors, stop, status

LOG=open('/memory/traj.jsonl','a',buffering=1)
RXLOG=open('/memory/radio_rx.log','a',buffering=1)

class PipeReader:
    def __init__(self,port,log=None):
        self.fd=os.open('/dev/robot/'+port,os.O_RDONLY|os.O_NONBLOCK)
        self.buf=b''; self.last=''; self.log=log; self.fresh=False
    def poll(self):
        self.fresh=False
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
                    self.last=s; self.fresh=True
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

def clean(v): return 3.0 if v<0 else v

state='run'
turn_tgt=0
scores=deque()
last_uturn=time.time()
last_tx=0
score=None

while True:
  try:
    l=lidar()
    if l is None: time.sleep(0.05); continue
    L=[clean(v) for v in l]
    front=min(L[15],L[0],L[1])
    odo.update()
    st=status(); goal_flag=st.get('goal','0')
    now=time.time()
    s5=p5.poll(); p4.poll()
    try: score=float(s5)
    except: pass
    if score is not None:
        scores.append((now,score))
        while scores and now-scores[0][0]>10: scores.popleft()
    LOG.write(json.dumps({'t':round(now,2),'x':round(odo.x,3),'y':round(odo.y,3),
      'h':round(odo.h or -1,1),'l':l,'g':goal_flag,'d5':s5,'st':state})+'\n')
    if goal_flag!='0':
        stop(); wr('d0','GOAL REACHED'); time.sleep(1); continue
    if now-last_tx> (0.5 if (score or 0)>0.85 else 1.5):
        wr('d0','HELLO A x=%.2f y=%.2f d5=%s'%(odo.x,odo.y,s5))
        last_tx=now
    # u-turn if signal falling
    if state=='run' and score is not None and len(scores)>15 and now-last_uturn>12:
        old=scores[0][1]
        if score<old-0.025:
            turn_tgt=((odo.h or 0)+180)%360
            state='turn'; last_uturn=now
            LOG.write(json.dumps({'ev':'uturn','from':old,'to':score})+'\n')
    if state=='run':
        if front<0.3:
            # junction: pick open direction; prefer beams roughly ahead, else any
            cands=[i for i in range(16) if L[i]>0.9 and not (5<=i<=11)]
            if not cands:
                cands=[i for i in range(16) if L[i]>0.6]
            if not cands:
                cands=[max(range(16),key=lambda k:L[k])]
            i=random.choice(cands)
            turn_tgt=((odo.h or 0)+22.5*i)%360
            state='turn'
        else:
            u=0.0
            # centering: push away from close side walls
            u+= 18*max(0,0.25-min(L[13],L[14],L[15]))/0.25   # left close -> turn right(+h)-> u negative? see below
            u-= 18*max(0,0.25-min(L[1],L[2],L[3]))/0.25
            # u>0 means turn toward beam-12 side (h decreasing): motors(base-u*?..)
            # careful: beams 13-15 are left(-h side)? beam i at h+22.5i; beam 15 ~ -22.5deg (h-), beam 1 ~ +22.5
            # if beams 13-15 close, obstacle on h- side -> turn h+ -> motors(+,-) -> u applied as motors(base+u,base-u) with u>0
            base=110 if front>1.5 else (60 if front>0.7 else 35)
            if score is not None and score>0.9: base=min(base,45)
            motors(base+u, base-u)
    elif state=='turn':
        err=((turn_tgt-(odo.h or 0)+180)%360)-180
        if abs(err)<10:
            state='run'
        else:
            sp=max(min(err*2.2,55),-55)
            if 0<=sp<16: sp=16
            if -16<sp<0: sp=-16
            motors(sp,-sp)
    time.sleep(0.08)
  except Exception as e:
    stop()
    LOG.write(json.dumps({'err':str(e)})+'\n')
    time.sleep(0.5)
