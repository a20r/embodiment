import time, math, os, threading, select
NB=os.O_NONBLOCK
P={}
for p in ('d0','d1','d2','d3','d5','d6','d8'):
    P[p]=os.open(f'/dev/robot/{p}', os.O_RDONLY|NB)
latest={p:'' for p in P}
pulses={'d1':0,'d6':0}
lock=threading.Lock()
def reader(p):
    buf=b''
    while True:
        r,_,_=select.select([P[p]],[],[],0.05)
        if r:
            try: d=os.read(P[p],65536)
            except BlockingIOError: continue
            if not d: continue
            buf+=d
            *lines,rest=buf.split(b'\n')
            buf=rest
            with lock:
                if lines:
                    latest[p]=lines[-1].decode()
                    for l in lines:
                        if p in ('d1','d6') and l.strip()==b'1': pulses[p]+=1
for p in P:
    threading.Thread(target=reader,args=(p,),daemon=True).start()
def get(p):
    with lock: return latest[p]
def take_pulses():
    with lock:
        out=dict(pulses); pulses['d1']=0; pulses['d6']=0
        return out
def f(x,d=0.0):
    try: return float(x)
    except: return d
def beams():
    s=get('d5')
    try: return [float(x) for x in s.split(',')]
    except: return None
def emit(d4,d7,cache=[None,None]):
    d4=int(round(d4)); d7=int(round(d7))
    if cache[0]!=d4: wr('d4',str(d4)); cache[0]=d4
    if cache[1]!=d7: wr('d7',str(d7)); cache[1]=d7
def wr(p,v):
    fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|NB)
    os.write(fd, f'{v}\n'.encode()); os.close(fd)
LOG=open('/bot/src/auto5.log','a',buffering=1)
ST=open('/bot/src/auto5_state.csv','a',buffering=1)
def log(m): LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
last8={}
t0=time.time(); cyc=0
rev_until=0; turn_until=0
log('v5 start')
while True:
    if os.path.exists('/bot/src/STOP'): emit(0,0); log('STOP'); break
    d4=0; d7c=0; mode='?'
    b=beams()
    if b is None: time.sleep(0.03); continue
    bc=[6.0 if x<0 else x for x in b]
    hd=f(get('d3')); sp=f(get('d2'))
    try: wz=f(get('d0').split(',')[2])
    except: wz=0
    d8=get('d8')
    pl=take_pulses()
    if pl['d1']: log(f'EVENT d1 x{pl["d1"]} hd={hd:.0f} v={sp:.2f} {d8}')
    try:
        p=dict(kv.split('=') for kv in d8.split())
        for k in ('lap','goal','best','last'):
            if p[k]!=last8.get(k):
                log(f'*** {k}: {last8.get(k)} -> {p[k]} (d8={d8})'); last8[k]=p[k]
    except: pass
    avgL=(bc[3]+bc[4]+bc[5])/3; avgR=(bc[11]+bc[12]+bc[13])/3
    FL=(bc[2]+bc[3])/2; FR=(bc[13]+bc[14])/2
    front=min(bc[15],bc[0],bc[1])
    t=time.time()
    flag=None
    try: flag=open('/bot/src/FLAG').read().strip()
    except: pass
    if t<turn_until:
        emit(20,85 if flag=='L' else -85); mode='TURNIN'; continue
    if t<rev_until:
        yaw=1 if FL>FR+0.2 else (-1 if FR>FL+0.2 else 0)
        emit(-45,-yaw*70); mode='REV'; d7c=-yaw*70
    else:
        steer=85*(avgL-avgR)+45*(FL-FR)-30*wz
        steer=max(-85,min(85,steer)); d7c=steer
        if front<0.45:
            rev_until=t+0.9; emit(-45,0); mode='TOREV'; continue
        # forced junction taking
        if flag in ('L','R'):
            opening=(avgL>1.5) if flag=='L' else (avgR>1.5)
            if opening and t>6:
                turn_until=t+2.2; log(f'TAKING {flag} opening aL={avgL:.2f} aR={avgR:.2f} hd={hd:.0f}')
                os.remove('/bot/src/FLAG'); continue
        aw=abs(wz)
        if aw>0.45 or front<0.9: d4=20; mode='BEND'
        elif aw>0.25 or front<1.4: d4=38; mode='FWD'
        else: d4=75; mode='FAST'
        emit(d4,d7c)
    cyc+=1
    ST.write(f'{time.time()-t0:.2f},{cyc},{hd:.1f},{sp:.2f},{wz:.2f},{d4},{d7c},{mode},{";".join(f"{x:.2f}" for x in b)},{d8}\n')
    if cyc%40==0: log(f'c{cyc} hd={hd:.0f} v={sp:.2f} front={front:.2f} aL={avgL:.2f} aR={avgR:.2f} {mode} {d8}')
    time.sleep(0.045)
