import time, math, os, threading, select, re, traceback
NB=os.O_NONBLOCK
P={p:os.open(f'/dev/robot/{p}', os.O_RDONLY|NB) for p in ('d0','d1','d2','d3','d5','d6','d8')}
latest={p:'' for p in P}; pulses={'d1':0,'d6':0}
lock=threading.Lock()
def reader(p):
    buf=b''
    while True:
        try:
            r,_,_=select.select([P[p]],[],[],0.05)
            if r:
                try: d=os.read(P[p],65536)
                except BlockingIOError: continue
                if not d: continue
                buf+=d
                *lines,rest=buf.split(b'\n'); buf=rest
                with lock:
                    if lines: latest[p]=lines[-1].decode()
                    for l in lines:
                        if p in ('d1','d6') and l.strip()==b'1': pulses[p]+=1
        except Exception: pass
for p in P: threading.Thread(target=reader,args=(p,),daemon=True).start()
def get(p):
    with lock: return latest[p]
def tp():
    with lock:
        o=dict(pulses); pulses['d1']=0; pulses['d6']=0; return o
def f(x,d=0.0):
    try: return float(x)
    except: return d
def wr(p,v):
    fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|NB); os.write(fd,f'{int(v)}\n'.encode()); os.close(fd)
def emit(d4,d7,cache=[None,None,None]):
    d4=int(round(d4)); d7=int(round(d7)); t=time.time()
    try:
        if cache[0]!=d4 or t-cache[2]>1.5: wr('d4',d4); cache[0]=d4
        if cache[1]!=d7 or t-cache[2]>1.5: wr('d7',d7); cache[1]=d7
        cache[2]=t
    except Exception:
        try: cache[0]=cache[1]=None
        except Exception: pass
LOG=open('/bot/src/auto8.log','a',buffering=1)
LAPLOG=open('/bot/src/LAPS.log','a',buffering=1)
CSV=open('/bot/src/auto8.csv','a',buffering=1)
def log(m):
    LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
def laplog(m):
    LAPLOG.write(f'[{time.time()%100000:8.1f}] {m}\n'); log('LAPBOOK '+m)
D8RE=re.compile(r'tick=(\d+) goal=(-?\d+) lap=(-?\d+) last=([-\d.]+) best=([-\d.]+)')
px=py=th=0.0; lastt=None
t0=time.time(); cyc=0; rev_until=0; rev_start=0; turn_until=0
lastopen={'L':0.0,'R':0.0}; stuck_since=None
prev={'lap':None,'goal':None}
hb=0; csvn=0
log('v8 start')
while True:
    try:
        if os.path.exists('/bot/src/STOP'):
            emit(0,0); time.sleep(0.2); lastt=None; continue
        s=get('d5')
        try: b=[float(x) for x in s.split(',')]
        except Exception: time.sleep(0.03); continue
        if len(b)!=16: time.sleep(0.03); continue
        bc=[6.0 if x<0 else min(x,5.99) for x in b]
        hd=f(get('d3')); sp=f(get('d2'))
        try: wz=f(get('d0').split(',')[2])
        except Exception: wz=0
        d8=get('d8'); pl=tp(); tick='0'
        t=time.time()
        # odometry
        if lastt is None: lastt=t
        dt=min(t-lastt,0.2); lastt=t
        th+=wz*dt
        px+=sp*math.cos(th)*dt; py+=sp*math.sin(th)*dt
        m=D8RE.search(d8)
        if m:
            tick,goal,lap,lastt_lap,best=m.groups()
            if prev['lap'] is not None and (lap!=prev['lap'] or goal!=prev['goal']):
                laplog(f'CHANGE lap {prev["lap"]}->{lap} goal {prev["goal"]}->{goal} last={lastt_lap} best={best} tick={tick} pose=({px:.2f},{py:.2f}) th={math.degrees(th)%360:.1f} hd={hd:.0f} v={sp:.2f}')
            prev['lap']=lap; prev['goal']=goal
        if pl['d1']: log(f'@@@ D1 x{pl["d1"]} pose=({px:.2f},{py:.2f}) hd={hd:.0f} v={sp:.2f} {d8}')
        if pl['d6']: log(f'~~~ D6 x{pl["d6"]} pose=({px:.2f},{py:.2f}) hd={hd:.0f} v={sp:.2f}')
        avgL=(bc[3]+bc[4]+bc[5])/3; avgR=(bc[11]+bc[12]+bc[13])/3
        FL=(bc[2]+bc[3])/2; FR=(bc[13]+bc[14])/2
        front=min(bc[15],bc[0],bc[1])
        d4=0; d7c=0; mode='?'
        if t<turn_until:
            fl=None
            try: fl=open('/bot/src/FLAG').read().strip()
            except Exception: pass
            emit(22, 85 if fl=='L' else -85); mode='TURNIN'; d7c=85 if fl=='L' else -85
        elif t<rev_until:
            yaw=1 if FL>FR+0.2 else (-1 if FR>FL+0.2 else 0)
            d7c=-yaw*70
            if front<0.5 and t-rev_start<4.0: rev_until=t+0.25
            emit(-45,d7c); mode='REV'
        else:
            if front<0.42:
                rev_until=t+1.1; rev_start=t; emit(-45,0); mode='TOREV'
            else:
                # stuck detection
                if sp*0+abs(sp)<0.035 and abs(wz)<0.1:
                    if stuck_since is None: stuck_since=t
                else: stuck_since=None
                if stuck_since is not None and t-stuck_since>1.6:
                    rev_until=t+1.3; rev_start=t; stuck_since=None
                    log(f'STUCK pose=({px:.2f},{py:.2f}) front={front:.2f} -> REV')
                    mode='UNSTICK'
                else:
                    steer=85*(avgL-avgR)+45*(FL-FR)-30*wz
                    d7c=max(-85,min(85,steer))
                    aw=abs(wz)
                    if aw>0.45 or front<0.9: d4=20; mode='BEND'
                    elif aw>0.25 or front<1.4: d4=38; mode='FWD'
                    else: d4=78; mode='FAST'
                    flag=None
                    try: flag=open('/bot/src/FLAG').read().strip()
                    except Exception: pass
                    if flag in ('L','R') and t-t0>5:
                        if flag=='L' and avgL>1.5 and t-lastopen['L']>4:
                            lastopen['L']=t; turn_until=t+2.2; mode='TAKE-L'
                            log(f'TAKING L hd={hd:.0f} aL={avgL:.2f} pose=({px:.1f},{py:.1f})')
                            os.remove('/bot/src/FLAG')
                        elif flag=='R' and avgR>1.5 and t-lastopen['R']>4:
                            lastopen['R']=t; turn_until=t+2.2; mode='TAKE-R'
                            log(f'TAKING R hd={hd:.0f} aR={avgR:.2f} pose=({px:.1f},{py:.1f})')
                            os.remove('/bot/src/FLAG')
                    emit(d4,d7c)
        emit(d4,d7c) if mode in ('?','TOREV') else None
        csvn+=1
        if csvn%4==0: CSV.write(f'{t-t0:.2f},{tick if m else 0},{px:.3f},{py:.3f},{math.degrees(th)%360:.1f},{hd:.1f},{sp:.3f},{wz:.3f},{d4},{d7c},{mode},{front:.2f},{avgL:.2f},{avgR:.2f}\n')
        cyc+=1
        if cyc%400==0: log(f'c{cyc} hd={hd:.0f} v={sp:.2f} pose=({px:.1f},{py:.1f}) th={math.degrees(th)%360:.0f} front={front:.2f} {mode} d8={d8}')
        if t-hb>2:
            hb=t
            with open('/bot/src/HB8','w') as h: h.write(f'{t:.1f} cyc={cyc} pose=({px:.1f},{py:.1f})\n')
        time.sleep(0.045)
    except Exception:
        try: log('EXC '+traceback.format_exc().replace('\n',' | ')[-400:])
        except Exception: pass
        try: emit(0,0)
        except Exception: pass
        time.sleep(0.05)
