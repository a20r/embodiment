import time, math, sys

def rdev(dev):
    for _ in range(8):
        try:
            with open('/dev/robot/'+dev) as f:
                s=f.read().strip()
            if s: return s.split('\n')[-1].strip()
        except: pass
        time.sleep(0.01)
    return ''

def w(dev,v):
    with open('/dev/robot/'+dev,'w') as f: f.write(str(int(v))+'\n')

def stop(): w('motor_left',0); w('motor_right',0)

def heading():
    s=rdev('heading')
    return float(s) if s else None

def lidar():
    s=rdev('lidar')
    if not s: return None
    v=[float(x) for x in s.split(',')]
    return [9.9 if x<0 else x for x in v]

def goal():
    s=rdev('status')
    return 'goal=1' in s

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d

def turn_to(target):
    # closed loop turn to absolute heading (deg, CCW positive)
    for _ in range(200):
        h=heading()
        if h is None: continue
        e=angdiff(target,h)
        if abs(e)<4:
            stop(); time.sleep(0.15)
            h=heading(); e=angdiff(target,h)
            if abs(e)<5: return True
            continue
        sp=max(40,min(140,abs(e)*2.5))
        if e>0: w('motor_left',-sp); w('motor_right',sp)
        else: w('motor_left',sp); w('motor_right',-sp)
        time.sleep(0.05)
    stop(); return False

TICKS=1900  # per meter
def enc():
    while True:
        try:
            l=rdev('encoder_left'); r=rdev('encoder_right')
            return int(l), int(r)
        except ValueError: pass

def forward(dist, target_h):
    l0,r0=enc()
    while True:
        l,r=enc()
        trav=((l-l0)+(r-r0))/2.0/TICKS
        if trav>=dist: break
        ld=lidar()
        if ld and ld[0]<0.20: break
        if rdev('bump_front')=='1':
            w('motor_left',-80); w('motor_right',-80); time.sleep(0.3); break
        h=heading()
        e=angdiff(target_h,h) if h is not None else 0
        # lateral centering using side beams
        lat=0
        if ld:
            L,R=ld[4],ld[12]
            if L<0.45 and R<0.45: lat=(L-R)*60
            elif L<0.45: lat=(L-0.24)*60
            elif R<0.45: lat=(0.24-R)*60
        corr=e*4+lat
        corr=max(-50,min(50,corr))
        base=130
        w('motor_left',base-corr); w('motor_right',base+corr)
        time.sleep(0.04)
    stop(); time.sleep(0.1)

CELL=0.5
log=open('/memory/run.log','a')
def L(*a):
    print(*a); log.write(' '.join(map(str,a))+'\n'); log.flush()

# snap heading to nearest 90
h=heading()
cur=round(h/90)%4*90
turn_to(cur)
L('start heading',h,'snap',cur)
t0=time.time()
steps=0
while time.time()-t0<600:
    if goal():
        L('GOAL after',steps,'steps'); stop(); sys.exit(0)
    if rdev('bump_front')=='1':
        w('motor_left',-90); w('motor_right',-90); time.sleep(0.8); stop(); time.sleep(0.2)
        turn_to(cur)
    ld=lidar()
    if not ld: continue
    F,Lft,R=ld[0],ld[4],ld[12]
    if Lft>0.4: choice='left'
    elif F>0.4: choice='straight'
    elif R>0.4: choice='right'
    else: choice='back'
    L('step',steps,'h',cur,'F',F,'L',Lft,'R',R,'->',choice)
    if choice=='left': cur=(cur+90)%360; turn_to(cur)
    elif choice=='right': cur=(cur-90)%360; turn_to(cur)
    elif choice=='back': cur=(cur+180)%360; turn_to(cur)
    forward(CELL,cur)
    steps+=1
    if goal():
        L('GOAL after',steps,'steps'); stop(); sys.exit(0)
L('timeout, no goal')
stop()
