import rc, time, math, sys

LOG=open('/bot/src/follow.log','a',buffering=1)
def log(*a):
    print(time.strftime('%H:%M:%S'),*a,file=LOG)

SIDE = sys.argv[1] if len(sys.argv)>1 else 'right'
# beam indices
F=[2,3,4]; 
if SIDE=='right':
    S=[6,7,8]; DIAG=5
else:
    S=[13,14,15]; DIAG=1

def med(vals):
    v=sorted(x for x in vals if x>=0)
    return v[len(v)//2] if v else 1.5

def sense():
    l=rc.lidar()
    front=med([l[i] for i in F])
    side=med([l[i] for i in S])
    diag=l[DIAG] if l[DIAG]>=0 else 1.5
    return l,front,side,diag

TARGET=0.18
BASE=45
x,y=0.0,0.0
laste=None
t0=time.time()
i=0
while True:
    i+=1
    l,front,side,diag=sense()
    bump = rc.readline('d5')=='1'
    g=rc.goal()
    if g:
        rc.stop(); log('GOAL REACHED', rc.status()); print('GOAL'); break
    # odometry
    e=rc.enc(); h=rc.heading()
    if laste:
        d=((e[0]-laste[0])+(e[1]-laste[1]))/2/333.0
        ang=math.radians(h)  # forward bearing guess
        x+=d*math.sin(ang); y+=d*math.cos(ang)
    laste=e
    if i%20==0:
        log(f'pos=({x:.2f},{y:.2f}) h={h:.0f} f={front:.2f} s={side:.2f} d={diag:.2f} bump={bump} lidar={[round(v,2) for v in l]}')
    if bump:
        log('bump! backing')
        rc.wheels(-40,-40); time.sleep(0.7)
        # turn away from side wall
        if SIDE=='right': rc.wheels(-40,40)
        else: rc.wheels(40,-40)
        time.sleep(0.8)
        continue
    if front<0.20:
        # turn away from followed side
        if SIDE=='right': rc.wheels(-35,35)
        else: rc.wheels(35,-35)
        time.sleep(0.1)
        continue
    err=side-TARGET
    err=max(-0.15,min(0.15,err))
    steer=err*250   # positive: too far from wall -> turn toward wall
    if diag<0.16: steer-= (0.16-diag)*400 if SIDE=='right' else -(0.16-diag)*400
    if SIDE=='right':
        rc.wheels(BASE+steer, BASE-steer)
    else:
        rc.wheels(BASE-steer, BASE+steer)
    time.sleep(0.08)
rc.stop()
