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
def scan():
    s=rd('d2')
    pts=[tuple(map(float,t.split(','))) for t in s.split(';') if t]
    return [(math.hypot(x,y), math.degrees(math.atan2(y,x))) for x,y,z in pts if True]
def clr(rp,lo,hi):
    rs=[r for r,a in rp if r>=0.12 and lo<=a<=hi]
    return min(rs) if rs else 3.0
def d0rate(n=8):
    c=0
    for _ in range(n):
        if rd('d0')=='1': c+=1
        time.sleep(0.05)
    return c/n
log=open('/tmp/probe_out.log','a',buffering=1)
t_end=time.time()+900
last_ping=0
H=0.0; h=float(rd('d4'))
state='find_gap'
gap_az=None
while time.time()<t_end:
    rp=scan()
    if state=='find_gap':
        # find widest-open 40deg window beyond body
        best=None
        for a in range(-180,180,10):
            c=clr(rp,a-20,a+20)
            if best is None or c>best[1]: best=(a,c)
        gap_az=best[0]; 
        log.write(f'gap at az={gap_az} clr={best[1]:.2f}\n')
        if best[1]>1.0:
            state='exit'
    elif state=='exit':
        err=(gap_az+180)%360-180
        # rotate toward gap (patient)
        if abs(err)>15:
            t0=time.time()
            while time.time()-t0<2.5:
                nh=float(rd('d4')); H+=(nh-h+180)%360-180; h=nh
                e2=err-H
                if abs(e2)<12: break
                d=max(-13,min(13,int(e2*0.5)))
                if d%2: d+=1
                for _ in range(3): wr('d1',d//2); wr('d7',-d//2); time.sleep(0.08)
            for _ in range(3): wr('d1',0); wr('d7',0); time.sleep(0.2)
        else:
            drive(12,12,1.0)
            for _ in range(3): wr('d1',0); wr('d7',0); time.sleep(0.2)
        rp=scan()
        fwd=clr(rp,-25,25)
        log.write(f'exit fwd={fwd:.2f}\n')
        if fwd>1.6:
            state='probe'; log.write('OUTSIDE - probing\n')
    elif state=='probe':
        r=d0rate(10)
        fl=rd('d3'); d5=rd('d5')
        log.write(f'PROBE d0rate={r:.2f} d5={d5} {fl}\n')
        if r>0.4 or 'here=1' in fl or 'goal=1' in fl:
            log.write('*** SIGNAL! HOLDING ***\n')
            open('/memory/EVENT.log','a').write(json.dumps({'t':time.time(),'d0rate':r,'fl':fl})+'\n')
            t_hold=time.time()+600
            while time.time()<t_hold:
                if time.time()-last_ping>2: wr('d8','A GOAL SPOT COME'); last_ping=time.time()
                log.write(f'HOLD {rd("d3")} d0={rd("d0")}\n')
                time.sleep(2)
            break
        # skirt: keep moving forward, steer to keep wall ~0.7-1m
        fwd=clr(rp,-25,25); L=clr(rp,15,60); R=clr(rp,-60,-15)
        diff=0
        if fwd<0.8:
            diff=12 if L>R else -12
        elif L<0.6: diff=-8
        elif R<0.6: diff=8
        drive(12+diff//2,12-diff//2,1.0)
        for _ in range(3): wr('d1',0); wr('d7',0); time.sleep(0.2)
    if time.time()-last_ping>4: wr('d8','A PROBE OUTSIDE PEN'); last_ping=time.time()
