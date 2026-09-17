import time, math, os
from drv import rd, wr
LOG=open('/bot/src/auto3.log','a',buffering=1)
ST=open('/bot/src/auto3_state.csv','a',buffering=1)
def log(m): LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
def f(x,d=0.0):
    try: return float(x)
    except: return d
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
log('v3 start')
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
    d1=rd('d1',0.02); d8=rd('d8',0.05)
    try:
        p=dict(kv.split('=') for kv in d8.split())
        for k in ('lap','goal','best','last'):
            if p[k]!=last[k]:
                log(f'*** {k}: {last[k]} -> {p[k]}  (d8={d8})'); last[k]=p[k]
    except: pass
    if d1=='1': log(f'EVENT d1=1 hd={hd:.0f} v={sp:.2f} {d8}')
    avgL=(bc[3]+bc[4]+bc[5])/3; avgR=(bc[11]+bc[12]+bc[13])/3
    FL=(bc[2]+bc[3])/2; FR=(bc[13]+bc[14])/2
    front=min(bc[15],bc[0],bc[1])
    t=time.time()
    if t<rev_until:
        yaw=1 if FL>FR+0.2 else (-1 if FR>FL+0.2 else 0)
        d7c=-yaw*70
        emit(-45,d7c); mode='REV'
    else:
        gap=(FL-FR)
        center=(avgL-avgR)
        steer=85*center+45*gap-30*wz
        steer=max(-80,min(80,steer))
        d7c=steer
        if front<0.45:
            rev_until=t+0.9; emit(-45,0); mode='TOREV'; continue
        if front<1.0: d4=16; mode='BEND'
        elif front>2.2: d4=50; mode='FAST'
        else: d4=32; mode='FWD'
        emit(d4,d7c)
    cyc+=1
    ST.write(f'{time.time()-t0:.2f},{cyc},{hd:.1f},{sp:.2f},{wz:.2f},{d4},{d7c},{mode},{";".join(f"{x:.2f}" for x in b)},{d8}\n')
    if cyc%40==0:
        log(f'c{cyc} hd={hd:.0f} v={sp:.2f} wz={wz:.2f} front={front:.2f} aL={avgL:.2f} aR={avgR:.2f} FL={FL:.2f} FR={FR:.2f} d4={d4} d7={d7c:.0f} {mode} {d8}')
    time.sleep(0.05)
