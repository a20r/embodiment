import math, subprocess, time, os, json
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
def d0rate(n=12):
    c=0
    for _ in range(n):
        if rd('d0')=='1': c+=1
        time.sleep(0.05)
    return c/n
def turn(Hcur, delta, h):
    # closed-loop rotate by delta deg, returns (H,h)
    H=Hcur; t0=time.time()
    while time.time()-t0<4:
        nh=float(rd('d4')); H+=(nh-h+180)%360-180; h=nh
        err=delta-H
        if abs(err)<7: break
        d=max(-12,min(12,int(err*0.5)))
        if d%2: d+=1
        for _ in range(3): wr('d1',d//2); wr('d7',-d//2); time.sleep(0.08)
    for _ in range(4): wr('d1',0); wr('d7',0)
    return H,h
log=open('/tmp/hunt2.log','a',buffering=1)
H=0.0; h=float(rd('d4'))
best_H=55.0
for step in range(120):
    # steer toward best_H
    err=(best_H-H+180)%360-180
    if abs(err)>8:
        H,h=turn(H,err,h)
    r=d0rate()
    # probe left/right occasionally to re-center bearing
    if step%4==2:
        H,h=turn(H,22,h); rl2=d0rate(); H,h=turn(H,-44,h); rneg=d0rate(); H,h=turn(H,22,h)
        if rl2>r: best_H=(best_H+22)%360; r=rl2
        if rneg>r: best_H=(best_H-44+22)%360; r=rneg
        H,h=turn(H,(best_H-H+180)%360-180,h)
    else:
        drive(16,16,1.0); H+=0  # fwd, heading drift ok
    fl=rd('d3')
    log.write(f'step{step} best_H={best_H:.0f} H={H:.0f} d0rate={r:.2f} {fl}\n')
    if 'here=1' in fl:
        log.write('GOAL HERE!!!\n')
        for _ in range(10): wr('d8','A AT GOAL COME')
        break
    if 'goal=1' in fl:
        log.write('GOAL FLAG!\n')
    time.sleep(0.2)
for _ in range(5): wr('d1',0); wr('d7',0)
