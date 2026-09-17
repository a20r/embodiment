import math, subprocess, time, os, json, collections
def wr(p,v):
    for att in range(3):
        try:
            fd=os.open(f'/dev/robot/{p}',os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd,f'{int(v)}\n'.encode()); os.close(fd); return
        except Exception: time.sleep(0.03)
def rd(p,tries=4):
    for _ in range(tries):
        r=subprocess.run(['timeout','0.3','cat',f'/dev/robot/{p}'],capture_output=True,text=True)
        if r.stdout.strip(): return r.stdout.strip()
        time.sleep(0.04)
    return '0'
def drive(a,b,dur,period=0.08):
    t0=time.time()
    while time.time()-t0<dur:
        wr('d1',a); wr('d7',b); time.sleep(period)
def scan():
    s=rd('d2')
    pts=[tuple(map(float,t.split(','))) for t in s.split(';') if t]
    return [(x,y) for x,y,z in pts]
def env_pts():
    return [(x,y) for x,y in scan() if math.hypot(x,y)>=0.12 and not (math.hypot(x,y)<0.55 and not (-50<=math.degrees(math.atan2(y,x))<=35))]
def circle_center(env):
    n=len(env)
    Sxx=sum(x*x for x,y in env); Syy=sum(y*y for x,y in env); Sxy=sum(x*y for x,y in env)
    Sx=sum(x for x,y in env); Sy=sum(y for x,y in env)
    Sxz=sum(x*(x*x+y*y) for x,y in env); Syz=sum(y*(x*x+y*y) for x,y in env); Sz=sum(x*x+y*y for x,y in env)
    A=[[Sxx,Sxy,Sx],[Sxy,Syy,Sy],[Sx,Sy,n]]; B=[Sxz,Syz,Sz]
    for i in range(3):
        piv=A[i][i]
        for j in range(i+1,3):
            f=A[j][i]/piv
            for k in range(3): A[j][k]-=f*A[i][k]
            B[j]-=f*B[i]
    X=[0,0,0]
    for i in (2,1,0):
        X[i]=(B[i]-sum(A[i][k]*X[k] for k in range(i+1,3)))/A[i][i]
    a_,b_,c_=X
    cx,cy=a_/2,b_/2
    return cx,cy,math.sqrt(max(c_+cx*cx+cy*cy,0))
def d0rate(n=10):
    c=0
    for _ in range(n):
        if rd('d0')=='1': c+=1
        time.sleep(0.05)
    return c/n
log=open('/tmp/lobe.log','a',buffering=1)
H=0.0; h=float(rd('d4'))
last_ping=0
for ray in [0, 40, -40, 80, -80]:
    # aim: re-fit center each ray, turn so center is BEHIND at -180+ray? we want to move along ray direction from center:
    # strategy: point robot AWAY from center rotated by ray => center should be at az = -180+ray in robot frame
    env=env_pts()
    cx,cy,R=circle_center(env)
    caz=math.degrees(math.atan2(cy,cx))
    err=(((-180+ray)-caz)+180)%360-180
    t0=time.time(); tgtH=H+err
    while time.time()-t0<5:
        nh=float(rd('d4')); H+=(nh-h+180)%360-180; h=nh
        e2=tgtH-H
        if abs(e2)<12: break
        d=max(-13,min(13,int(e2*0.5)))
        if d%2: d+=1
        for _ in range(3): wr('d1',d//2); wr('d7',-d//2); time.sleep(0.08)
    for _ in range(3): wr('d1',0); wr('d7',0); time.sleep(0.2)
    log.write(f'ray{ray} aimed, H={H:.0f}\n')
    # creep outward up to 0.5m in 10cm steps, d0rate each step
    for k in range(5):
        drive(10,10,1.3)
        for _ in range(3): wr('d1',0); wr('d7',0)
        time.sleep(0.3)
        r=d0rate()
        fl=rd('d3')
        log.write(f'ray{ray} step{k} d0rate={r:.2f} {fl}\n')
        print(f'ray{ray} step{k} d0rate={r:.2f}', flush=True)
        if r>=0.7 or 'here=1' in fl:
            log.write('*** SIGNAL HOLD ***\n')
            t_hold=time.time()+420
            while time.time()<t_hold:
                if time.time()-last_ping>2: wr('d8','A GOAL SPOT COME'); last_ping=time.time()
                time.sleep(1.5)
            raise SystemExit
    # re-center: aim back toward center (center az should be ~ray+180), creep 2 steps
    env=env_pts()
    cx,cy,R=circle_center(env)
    caz=math.degrees(math.atan2(cy,cx))
    err=((caz)+180)%360-180
    t0=time.time(); tgtH=H+err
    while time.time()-t0<4:
        nh=float(rd('d4')); H+=(nh-h+180)%360-180; h=nh
        e2=tgtH-H
        if abs(e2)<12: break
        d=max(-13,min(13,int(e2*0.5)))
        if d%2: d+=1
        for _ in range(3): wr('d1',d//2); wr('d7',-d//2); time.sleep(0.08)
    for _ in range(3): wr('d1',0); wr('d7',0); time.sleep(0.2)
    drive(10,10,1.2)
    for _ in range(3): wr('d1',0); wr('d7',0)
for _ in range(4): wr('d1',0); wr('d7',0)
