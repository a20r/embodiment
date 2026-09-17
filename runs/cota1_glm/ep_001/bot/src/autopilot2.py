import time, math, os
from drv import rd, wr
LOG=open('/bot/src/auto2.log','a',buffering=1)
ST=open('/bot/src/auto2_state.csv','a',buffering=1)
def log(m):
    LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
def f(x,d=0.0):
    try: return float(x)
    except: return d
def beams():
    s=rd('d5',0.08)
    try: return [float(x) for x in s.split(',')]
    except: return None
last={'lap':None,'goal':None,'best':None,'last':None}
def emit(d4,d7,cache=[None,None]):
    if cache[0]!=d4: wr('d4',str(int(round(d4)))); cache[0]=d4
    if cache[1]!=d7: wr('d7',str(int(round(d7)))); cache[1]=d7
t0=time.time(); cyc=0
log('v2 start')
stuck_t=0; last_modes=[]
while True:
    if os.path.exists('/bot/src/STOP'):
        emit(0,0); log('STOP'); break
    b=beams()
    if b is None: time.sleep(0.03); continue
    bc=[6.0 if x<0 else x for x in b]
    hd=f(rd('d3',0.02)); sp=f(rd('d2',0.02))
    try: wz=f(rd('d0',0.02).split(',')[2])
    except: wz=0
    d1=rd('d1',0.02); d6=rd('d6',0.02); d8=rd('d8',0.05)
    try:
        p=dict(kv.split('=') for kv in d8.split())
        for k in ('lap','goal','best','last'):
            if p[k]!=last[k]:
                log(f'*** {k}: {last[k]} -> {p[k]}  (d8={d8})'); last[k]=p[k]
    except: pass
    if d1=='1': log(f'EVENT d1=1 hd={hd:.0f} v={sp:.2f} {d8}')
    dL=min(bc[3],bc[4],bc[5]); dR=min(bc[11],bc[12],bc[13])
    front=min(bc[15],bc[0],bc[1])
    FL=min(bc[2],bc[3]); FR=min(bc[13],bc[14])
    center=(dL-dR)
    yaw_sign=0
    if FL>FR+0.18: yaw_sign=1
    elif FR>FL+0.18: yaw_sign=-1
    if front<0.30:
        d4=-45; d7=(-yaw_sign*70) if yaw_sign else 0; mode='REV'
    elif front<0.80:
        d4=16; d7=yaw_sign*55+max(-30,min(30,center*80)); mode='BEND'
    else:
        d4=45 if front>1.5 else (30 if front>1.0 else 20)
        d7=max(-70,min(70,center*110-25*wz)); mode='FWD'
    # stuck detection
    if mode!='REV' and abs(sp)<0.03: stuck_t+=0.05
    else: stuck_t=0
    if stuck_t>2.5:
        emit(-45, 40); log(f'STUCK wiggle hd={hd:.0f} front={front:.2f}')
        time.sleep(0.8); stuck_t=0
        continue
    emit(d4,d7)
    cyc+=1
    ST.write(f'{time.time()-t0:.2f},{cyc},{hd:.1f},{sp:.2f},{wz:.2f},{d4},{d7},{mode},{";".join(f"{x:.2f}" for x in b)},{d8}\n')
    if cyc%40==0:
        log(f'c{cyc} hd={hd:.0f} v={sp:.2f} front={front:.2f} dL={dL:.2f} dR={dR:.2f} FL={FL:.2f} FR={FR:.2f} d4={d4} d7={d7:.0f} {mode} {d8}')
    time.sleep(0.05)
