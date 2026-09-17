import time, math, os
from drv import rd, wr
LOG=open('/bot/src/auto4.log','a',buffering=1)
ST=open('/bot/src/auto4_state.csv','a',buffering=1)
def log(m): LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
def f(x,d=0.0):
    try: return float(x)
    except: return d
def raw(p,wait=0.08):
    # return ALL lines available (to catch short pulses)
    import select
    fd=os.open(f'/dev/robot/{p}', os.O_RDONLY|os.O_NONBLOCK)
    out=b''; t0=time.time()
    while time.time()-t0<wait:
        r,_,_=select.select([fd],[],[],0.03)
        if r:
            try:
                d=os.read(fd,65536)
                if d: out+=d
            except BlockingIOError: pass
    os.close(fd)
    return out.decode()
def beams():
    s=rd('d5',0.08)
    try: return [float(x) for x in s.split(',')]
    except: return None
def emit(d4,d7,cache=[None,None]):
    d4=int(round(d4)); d7=int(round(d7))
    if cache[0]!=d4: wr('d4',str(d4)); cache[0]=d4
    if cache[1]!=d7: wr('d7',str(d7)); cache[1]=d7
last={'lap':None,'goal':None,'best':None,'last':None}
t0=time.time(); cyc=0
log('v4 start')
rev_until=0
while True:
    if os.path.exists('/bot/src/STOP'):
        emit(0,0); log('STOP'); break
    b=beams()
    if b is None: time.sleep(0.03); continue
    bc=[6.0 if x<0 else x for x in b]
    hd=f(rd('d3',0.02)); sp=f(rd('d2',0.02))
    try: wz=f(rd('d0',0.02).split(',')[2])
    except: wz=0
    d1raw=raw('d1',0.03); d8=rd('d8',0.05)
    if '1' in [l.strip() for l in d1raw.split('\n') if l.strip()]:
        log(f'EVENT d1=1 hd={hd:.0f} v={sp:.2f} wz={wz:.2f} {d8}')
    try:
        p=dict(kv.split('=') for kv in d8.split())
        for k in ('lap','goal','best','last'):
            if p[k]!=last[k]:
                log(f'*** {k}: {last[k]} -> {p[k]}  (d8={d8})'); last[k]=p[k]
    except: pass
    avgL=(bc[3]+bc[4]+bc[5])/3; avgR=(bc[11]+bc[12]+bc[13])/3
    FL=(bc[2]+bc[3])/2; FR=(bc[13]+bc[14])/2
    front=min(bc[15],bc[0],bc[1])
    t=time.time()
    if t<rev_until:
        yaw=1 if FL>FR+0.2 else (-1 if FR>FL+0.2 else 0)
        d7c=-yaw*70
        emit(-45,d7c); mode='REV'
    else:
        steer=85*(avgL-avgR)+45*(FL-FR)-30*wz
        steer=max(-85,min(85,steer))
        d7c=steer
        if front<0.45:
            rev_until=t+0.9; emit(-45,0); mode='TOREV'; continue
        # speed: corner-limited by |wz| and clearance
        aw=abs(wz)
        if aw>0.45 or front<0.9: d4=20; mode='BEND'
        elif aw>0.25 or front<1.4: d4=38; mode='FWD'
        else: d4=75; mode='FAST'
        emit(d4,d7c)
    cyc+=1
    ST.write(f'{time.time()-t0:.2f},{cyc},{hd:.1f},{sp:.2f},{wz:.2f},{d4},{d7c},{mode},{";".join(f"{x:.2f}" for x in b)},{d8}\n')
    if cyc%40==0:
        log(f'c{cyc} hd={hd:.0f} v={sp:.2f} wz={wz:.2f} front={front:.2f} d4={d4} d7={d7c:.0f} {mode} {d8}')
    time.sleep(0.045)
