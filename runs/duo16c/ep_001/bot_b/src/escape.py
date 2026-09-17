import math, collections, subprocess, time, os
def wr(p,v):
    try:
        fd=os.open(f'/dev/robot/{p}',os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd,f'{int(v)}\n'.encode()); os.close(fd)
    except Exception: pass
def rd(p,tries=4):
    for _ in range(tries):
        r=subprocess.run(['timeout','0.35','cat',f'/dev/robot/{p}'],capture_output=True,text=True)
        if r.stdout.strip(): return r.stdout.strip()
        time.sleep(0.05)
    return '0'
def drive(a,b,dur):
    t0=time.time()
    while time.time()-t0<dur:
        wr('d1',a); wr('d7',b); time.sleep(0.08)
def scan():
    s=rd('d2')
    pts=[tuple(map(float,t.split(','))) for t in s.split(';') if t]
    return [(math.hypot(x,y), math.degrees(math.atan2(y,x)), z) for x,y,z in pts]
def clearance(rpts, lo, hi):
    rs=[r for r,a,z in rpts if r>=0.14 and lo<=a<=hi]
    return min(rs) if rs else 3.0
def min_clear_all(rpts):
    # clearance in 12 sectors of 30 deg
    return min(clearance(rpts, a, a+30) for a in range(-180,180,30))
log=open('/tmp/escape.log','a',buffering=1)
for step in range(60):
    rpts=scan()
    fwd=clearance(rpts,-18,18)
    back=clearance(rpts,162,198)
    allc=min_clear_all(rpts)
    log.write(f'step{step} fwd={fwd:.2f} back={back:.2f} minall={allc:.2f}\n')
    if allc>0.8:
        log.write('ESCAPED\n'); break
    if fwd>0.5:
        drive(16,16,0.9)
    elif back>0.5:
        drive(-16,-16,0.9)
    else:
        # wiggle: tiny reverse + micro turn
        drive(-12,-12,0.5); drive(8,-8,0.6)
    drive(0,0,0.2)
drive(0,0,0.3)
log.write('done\n')
