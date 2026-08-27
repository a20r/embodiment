import rc, time, math, sys
from collections import deque

LOG=open('/bot/src/follow.log','a',buffering=1)
def log(*a): print(time.strftime('%H:%M:%S'),*a,file=LOG)

SIDE = sys.argv[1] if len(sys.argv)>1 else 'right'
F=[2,3,4]
if SIDE=='right': S=[6,7,8]; DIAG=5
else: S=[13,14,15]; DIAG=1

def med(vals):
    v=sorted(x for x in vals if x>=0)
    return v[len(v)//2] if v else 1.5

TARGET=0.18; BASE=45
x,y=0.0,0.0; laste=None
hist=deque()
i=0
log(f'=== start2 side={SIDE}')
def odom():
    global x,y,laste
    e=rc.enc(); h=rc.heading()
    if laste:
        d=((e[0]-laste[0])+(e[1]-laste[1]))/2/333.0
        ang=math.radians(h)
        x+=d*math.sin(ang); y+=d*math.cos(ang)
    laste=e
    return h

def check_goal():
    if rc.goal():
        rc.stop(); log('GOAL REACHED', rc.status()); print('GOAL', flush=True)
        raise SystemExit

while True:
    i+=1
    l=rc.lidar()
    front=med([l[j] for j in F]); side=med([l[j] for j in S])
    diag=l[DIAG] if l[DIAG]>=0 else 1.5
    bump = rc.readline('d5')=='1'
    check_goal()
    h=odom()
    now=time.time()
    hist.append((now,x,y))
    while hist and hist[0][0]<now-35: hist.popleft()
    if i%20==0:
        log(f'pos=({x:.2f},{y:.2f}) h={h:.0f} f={front:.2f} s={side:.2f} bump={bump} lidar={[round(v,2) for v in l]}')
    old=[p for p in hist if p[0]<now-25]
    if old and (x-old[0][1])**2+(y-old[0][2])**2 < 0.8**2 and now-hist[0][0]>25:
        log(f'LOOP DETECTED at ({x:.2f},{y:.2f}) - escaping')
        l2=rc.lidar()
        best=max(range(16), key=lambda j: l2[j] if l2[j]>=0 else 0)
        tgt=(h+(best-3)*22.5)%360
        rc.turn_to(tgt,speed=50)
        rc.wheels(BASE,BASE)
        t0=time.time()
        while time.time()-t0<12:
            l3=rc.lidar()
            f3=med([l3[j] for j in F])
            check_goal()
            if f3<0.25 or rc.readline('d5')=='1': break
            time.sleep(0.08)
        rc.stop()
        odom(); hist.clear()
        log(f'escape done at ({x:.2f},{y:.2f})')
        continue
    if bump:
        log('bump! backing')
        rc.wheels(-40,-40); time.sleep(0.7)
        if SIDE=='right': rc.wheels(-40,40)
        else: rc.wheels(40,-40)
        time.sleep(0.8); continue
    if front<0.20:
        if SIDE=='right': rc.wheels(-35,35)
        else: rc.wheels(35,-35)
        time.sleep(0.1); continue
    err=max(-0.15,min(0.15,side-TARGET))
    steer=err*250
    if diag<0.16: steer-=(0.16-diag)*400 if SIDE=='right' else -(0.16-diag)*400
    if SIDE=='right': rc.wheels(BASE+steer,BASE-steer)
    else: rc.wheels(BASE-steer,BASE+steer)
    time.sleep(0.08)
