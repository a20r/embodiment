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
def d0rate(n=14):
    c=0
    for _ in range(n):
        if rd('d0')=='1': c+=1
        time.sleep(0.06)
    return c/n
log=open('/tmp/hunt.log','a',buffering=1)
H=0.0; h=float(rd('d4'))
# sample current heading
r0=d0rate()
log.write(f'H=0 d0rate={r0:.2f}\n')
print('H=0 d0rate', r0)
best=(r0,0.0)
for k in range(6):
    # rotate +30 closed loop
    t0=time.time()
    while time.time()-t0<4:
        nh=float(rd('d4')); H+=(nh-h+180)%360-180; h=nh
        err=30*(k+1)-H
        if abs(err)<8: break
        d=max(-12,min(12,int(err*0.5)))
        if d%2: d+=1
        for _ in range(3): wr('d1',d//2); wr('d7',-d//2); time.sleep(0.08)
    for _ in range(4): wr('d1',0); wr('d7',0)
    time.sleep(0.3)
    r=d0rate()
    log.write(f'H={H:.0f} d4={h:.1f} d0rate={r:.2f}\n')
    print(f'H={H:.0f} d0rate={r:.2f}')
    if r>best[0]: best=(r,H)
    if r0==0 and r>0: r0=r
json.dump({'best_rate':best[0],'best_H':best[1]}, open('/tmp/hunt_best.json','w'))
log.write(f'BEST rate={best[0]:.2f} H={best[1]:.0f}\n')
for _ in range(5): wr('d1',0); wr('d7',0)
