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
def clr(rpts, lo, hi):
    rs=[r for r,a,z in rpts if r>=0.14 and lo<=a<=hi]
    return min(rs) if rs else 3.0
log=open('/tmp/escape2.log','a',buffering=1)
for step in range(120):
    rpts=scan()
    fwd=clr(rpts,-15,15)
    back=clr(rpts,165,195)
    # steering: compare left-front vs right-front clearance
    L=clr(rpts,10,45); R=clr(rpts,-45,-10)
    allc=min(clr(rpts,a,a+30) for a in range(-180,180,30))
    if allc>0.85:
        log.write(f'step{step} ESCAPED allc={allc:.2f}\n'); break
    if fwd>max(back,0.35):
        # gentle arc toward more open side
        if L>R: drive(18,12,0.9)
        else: drive(12,18,0.9)
        log.write(f'step{step} fwdArc fwd={fwd:.2f} L={L:.2f} R={R:.2f}\n')
    elif back>0.35:
        if L>R: drive(-12,-18,0.9)
        else: drive(-18,-12,0.9)
        log.write(f'step{step} backArc back={back:.2f} L={L:.2f} R={R:.2f}\n')
    else:
        drive(-14,-14,0.6)
        log.write(f'step{step} backup fwd={fwd:.2f} back={back:.2f}\n')
    drive(0,0,0.15)
drive(0,0,0.3)
log.write('done\n')
