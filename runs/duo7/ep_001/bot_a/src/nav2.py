import robot, time, math, json, os

CPM = 1500.0
INF = 9.9
SPEED_FAST = 85
SPEED_SLOW = 40

def lid():
    L = robot.lidar()
    return [x if x and x>0 else INF for x in L]

class Nav:
    def __init__(self):
        self.x=0.0; self.y=0.0
        self.e=robot.enc(); self.h=robot.heading()
        # resume pose if recent
        try:
            p=json.load(open('/memory/pose.json'))
            if time.time()-p['t']<3600:
                self.x,self.y=p['x'],p['y']
        except: pass
        self.visits={}
        try:
            for k,v in json.load(open('/memory/visits.json')).items():
                a,b=k.split(','); self.visits[(int(a),int(b))]=v
        except: pass
    def update(self):
        e=robot.enc(); h=robot.heading()
        d=((e[0]-self.e[0])+(e[1]-self.e[1]))/2.0/CPM
        self.e=e; self.h=h
        r=math.radians(h)
        self.x+=d*math.sin(r); self.y+=d*math.cos(r)
        return h
    def save(self):
        json.dump({'t':time.time(),'x':self.x,'y':self.y},open('/memory/pose.json','w'))
        json.dump({'%d,%d'%k:v for k,v in self.visits.items()},open('/memory/visits.json','w'))
    def cell(self,dx=0,dy=0):
        return (round((self.x+dx)/0.5), round((self.y+dy)/0.5))
    def mark(self):
        c=self.cell(); self.visits[c]=self.visits.get(c,0)+1

nav=Nav()
trail=open('/memory/trail2.jsonl','a')
t0=time.time()
last_beacon=0; last_log=0

def housekeeping(front=None):
    global last_beacon,last_log
    now=time.time()
    st=robot.status() or ''
    if 'goal=1' in st:
        robot.motors(0,0)
        with open('/memory/GOAL_FOUND.txt','a') as f:
            f.write(json.dumps({'t':now,'x':nav.x,'y':nav.y,'st':st})+'\n')
        print('GOAL FOUND at %.2f %.2f'%(nav.x,nav.y),flush=True)
        while True:
            robot.motors(0,0)
            robot.tx('GOALFOUND alpha waiting on goal')
            m=robot.readline('d4',0.1)
            if m:
                with open('/memory/radio_rx.log','a') as f: f.write('%f %s\n'%(time.time(),m))
            time.sleep(8)
    if now-last_beacon>3:
        robot.tx('PING from=alpha x=%.2f y=%.2f'%(nav.x,nav.y))
        last_beacon=now
    m=robot.readline('d4',0.03)
    if m:
        with open('/memory/radio_rx.log','a') as f: f.write('%f %s\n'%(now,m))
        print('RX:',m,flush=True)
    if now-last_log>1.0:
        trail.write(json.dumps({'t':round(now-t0,1),'x':round(nav.x,2),'y':round(nav.y,2),'h':round(nav.h,1),'L':[round(v,2) for v in lid()]})+'\n')
        trail.flush(); nav.save(); last_log=now

def turn_to(target):
    while True:
        h=nav.update()
        d=robot.angdiff(target,h)
        if abs(d)<=4:
            robot.motors(0,0); return
        s=max(6,min(40,abs(d)*0.8))
        s=s if d>0 else -s
        robot.motors(s,-s)
        time.sleep(0.06)

def card_open(L,h):
    """distances toward N,E,S,W using nearest beams"""
    out={}
    for name,ang in (('N',0),('E',90),('S',180),('W',270)):
        i=round(((ang-h)%360)/22.5)%16
        out[name]=min(L[i],L[(i+1)%16],L[(i-1)%16])
        out[name+'0']=L[i]
    return out

def drive(H, watch_side=True):
    """drive along cardinal heading H until junction; returns reason"""
    dist=0.0; e0=sum(robot.enc())/2.0
    right_was_closed=False; left_was_closed=False
    off=0.0
    while True:
        h=nav.update(); L=lid()
        housekeeping()
        e1=sum(robot.enc())/2.0
        dist=(e1-e0)/CPM
        front=L[0]
        if min(L[15],L[1])<0.17: front=min(front,0.2)
        right=min(L[3],L[4],L[5]); left=min(L[11],L[12],L[13])
        if front<0.30:
            robot.motors(0,0); return 'front',dist
        if L[4]<0.35: right_was_closed=True
        if L[12]<0.35: left_was_closed=True
        if watch_side and left_was_closed and dist>0.15 and L[12]>0.55:
            ee=sum(robot.enc())/2.0
            robot.motors(28,28)
            while sum(robot.enc())/2.0-ee < 0.12*CPM:
                time.sleep(0.05); nav.update()
            robot.motors(0,0); return 'left_open',dist
        if watch_side and right_was_closed and dist>0.15 and L[4]>0.55:
            # opening on right: advance a bit to center then stop
            ee=sum(robot.enc())/2.0
            robot.motors(28,28)
            while sum(robot.enc())/2.0-ee < 0.12*CPM:
                time.sleep(0.05); nav.update()
            robot.motors(0,0); return 'right_open',dist
        err=robot.angdiff(H,h)
        steer=max(-12,min(12,1.2*err))
        if right<0.5 and left<0.5:
            steer+=max(-6,min(6,25*(right-left)))
        elif right<0.30: steer-=4
        elif left<0.30: steer+=4
        sp=SPEED_FAST if front>0.65 else SPEED_SLOW
        robot.motors(sp+steer,sp-steer)
        time.sleep(0.05)

# main loop
heading_card=None
while True:
    h=nav.update(); L=lid()
    housekeeping()
    nav.mark()
    c=card_open(L,h)
    # candidate order: right-hand rule relative to current cardinal
    if heading_card is None:
        order=['N','E','S','W']
    else:
        idx='NESW'.index(heading_card)
        order=['NESW'[(idx+1)%4],'NESW'[idx],'NESW'[(idx+3)%4],'NESW'[(idx+2)%4]]
    # filter open ones
    open_dirs=[d for d in order if c[d+'0']>0.5 and c[d]>0.22]
    if not open_dirs:
        open_dirs=[max('NESW',key=lambda d:c[d])]
    # prefer least-visited next cell
    def nextcell(d):
        dx={'N':0,'E':0.5,'S':0,'W':-0.5}[d]; dy={'N':0.5,'E':0,'S':-0.5,'W':0}[d]
        return nav.visits.get(nav.cell(dx,dy),0)
    best=min(open_dirs,key=lambda d:(nextcell(d),order.index(d)))
    print('junction pos=%.2f,%.2f h=%.0f N=%.2f E=%.2f S=%.2f W=%.2f open=%s best=%s'%(
        nav.x,nav.y,h,c['N0'],c['E0'],c['S0'],c['W0'],open_dirs,best),flush=True)
    heading_card=best
    ang={'N':0,'E':90,'S':180,'W':270}[best]
    turn_to(ang)
    reason,dist=drive(ang)
    print('drove %s %.2f stop=%s pos=%.2f,%.2f'%(best,dist,reason,nav.x,nav.y),flush=True)
    if reason=='front' and dist<0.06:
        dx={'N':0,'E':0.5,'S':0,'W':-0.5}[best]; dy={'N':0.5,'E':0,'S':-0.5,'W':0}[best]
        cc=nav.cell(dx,dy)
        nav.visits[cc]=nav.visits.get(cc,0)+3
        robot.motors(-25,-25); time.sleep(0.5); robot.motors(0,0); nav.update()
