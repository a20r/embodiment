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
def env_pts():
    s=rd('d2')
    pts=[(float(t.split(',')[0]), float(t.split(',')[1])) for t in s.split(';') if t]
    return [(x,y) for x,y in pts if math.hypot(x,y)>=0.12 and not (math.hypot(x,y)<0.55 and not (-50<=math.degrees(math.atan2(y,x))<=35))]
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
    r=math.sqrt(max(c_+cx*cx+cy*cy,0))
    return cx,cy,r
log=open('/tmp/aim.log','a',buffering=1)
H=0.0; h=float(rd('d4'))
for it in range(12):
    env=env_pts()
    cx,cy,R=circle_center(env)
    dist=math.hypot(cx,cy); az=math.degrees(math.atan2(cy,cx))
    log.write(f'it{it} center=({cx:.2f},{cy:.2f}) R={R:.2f} dist={dist:.2f} az={az:.0f} H={H:.0f}\n')
    print(f'it{it} dist={dist:.2f} az={az:.0f} H={H:.0f}', flush=True)
    if dist<0.12:
        log.write('CENTER REACHED\n'); print('CENTER REACHED'); break
    # rotate toward az (patient: up to 4 attempts of 3s)
    err=az
    for att in range(3):
        t0=time.time(); tgtH=H+err
        while time.time()-t0<2.5:
            nh=float(rd('d4')); H+=(nh-h+180)%360-180; h=nh
            e2=tgtH-H
            if abs(e2)<10: break
            d=max(-13,min(13,int(e2*0.5)))
            if d%2: d+=1
            for _ in range(3): wr('d1',d//2); wr('d7',-d//2); time.sleep(0.08)
        for _ in range(3): wr('d1',0); wr('d7',0)
        time.sleep(0.3)
        env=env_pts()
        cx,cy,R=circle_center(env)
        dist=math.hypot(cx,cy); az=math.degrees(math.atan2(cy,cx))
        err=az
        if abs(err)<15: break
    # creep forward up to 0.35m in 4 steps, refit each
    for k in range(4):
        drive(10,10,0.7)
        for _ in range(3): wr('d1',0); wr('d7',0)
        env=env_pts()
        cx,cy,R=circle_center(env)
        dist=math.hypot(cx,cy)
        if dist<0.1: break
        if dist>0.5:  # overshoot/lost: back
            drive(-10,-10,0.5); break
        az2=math.degrees(math.atan2(cy,cx))
        if abs(az2)>25: break  # need re-aim
        time.sleep(0.2)
    fl=rd('d3')
    if 'here=1' in fl or 'goal=1' in fl:
        log.write(f'FLAG! {fl}\n'); print('FLAG!', fl); break
for _ in range(4): wr('d1',0); wr('d7',0)
