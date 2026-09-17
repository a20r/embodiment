import time, math, os, threading, select
NB=os.O_NONBLOCK
P={p:os.open(f'/dev/robot/{p}', os.O_RDONLY|NB) for p in ('d0','d1','d2','d3','d5','d6','d8')}
latest={p:'' for p in P}; pulses={'d1':0,'d6':0}
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
            *lines,rest=buf.split(b'\n'); buf=rest
            with lock:
                if lines:
                    latest[p]=lines[-1].decode()
                    for l in lines:
                        if p in ('d1','d6') and l.strip()==b'1': pulses[p]+=1
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
    fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|NB)
    os.write(fd,f'{v}\n'.encode()); os.close(fd)
def emit(d4,d7,cache=[None,None]):
    d4=int(round(d4)); d7=int(round(d7))
    if cache[0]!=d4: wr('d4',str(d4)); cache[0]=d4
    if cache[1]!=d7: wr('d7',str(d7)); cache[1]=d7
# ---- SLAM ----
MS=320; RES=0.1; OFF=MS//2
grid=[[0]*MS for _ in range(MS)]  # 0 unk, 1 free, 2 occ
def w2g(x,y): 
    i=int(y/RES)+OFF; j=int(x/RES)+OFF
    return i,j
def raycast(px,py,th,r):
    n=int(r/RES)
    cx,cy=math.cos(th),math.sin(th)
    for k in range(1,n):
        x=px+cx*k*RES; y=py+cy*k*RES
        i,j=w2g(x,y)
        if 0<=i<MS and 0<=j<MS:
            if k*RES>=r-RES: grid[i][j]=2
            elif grid[i][j]==0: grid[i][j]=1
        else: break
px=py=0.0; th=0.0; lastt=None
def slam_update(b,wz,sp,t):
    global px,py,th,lastt
    if lastt is None: lastt=t; return
    dt=min(t-lastt,0.2); lastt=t
    th+=wz*dt
    px+=sp*math.cos(th)*dt; py+=sp*math.sin(th)*dt
    for i in range(16):
        r=b[i]
        if r<0: continue
        r=min(r,5.9)
        raycast(px,py,th+i*math.pi/8,r)
def dump_map():
    with open('/bot/src/map.txt','w') as f:
        for i in range(MS-1,-1,-2):
            row=''
            for j in range(0,MS,2):
                v=max(grid[i][j],grid[i][j-1] if j>0 else 0, grid[i-1][j] if i>0 else 0, grid[i-1][j-1] if i>0 and j>0 else 0)
                row += '#' if v==2 else ('.' if v==1 else ' ')
            f.write(row+'\n')
        f.write(f'pose {px:.2f} {py:.2f} th {math.degrees(th)%360:.1f}\n')
LOG=open('/bot/src/auto7.log','a',buffering=1)
def log(m): LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
SC=open('/bot/src/corridors.log','a',buffering=1)
t0=time.time(); cyc=0; rev_until=0; turn_until=0
lastopen={'L':0.0,'R':0.0,'CORR':-99}
log('v7 start')
mapt=0
while True:
    if os.path.exists('/bot/src/STOP'): emit(0,0); log('STOP'); break
    s=get('d5')
    try: b=[float(x) for x in s.split(',')]
    except: time.sleep(0.03); continue
    if len(b)!=16: time.sleep(0.03); continue
    bc=[6.0 if x<0 else x for x in b]
    hd=f(get('d3')); sp=f(get('d2'))
    try: wz=f(get('d0').split(',')[2])
    except: wz=0
    d8=get('d8'); pl=tp()
    if pl['d1']: log(f'@@@ D1 PULSE x{pl["d1"]} hd={hd:.0f} v={sp:.2f} pose=({px:.1f},{py:.1f}) {d8}')
    t=time.time()
    slam_update(bc,wz,sp,t)
    if t-mapt>6: mapt=t; dump_map()
    avgL=(bc[3]+bc[4]+bc[5])/3; avgR=(bc[11]+bc[12]+bc[13])/3
    FL=(bc[2]+bc[3])/2; FR=(bc[13]+bc[14])/2
    front=min(bc[15],bc[0],bc[1])
    d4=0; d7c=0; mode='?'
    if 0.38<avgL<0.68 and 0.38<avgR<0.68 and abs(avgL-avgR)<0.12 and bc[0]>2.2 and bc[15]>2.2 and bc[1]>1.5 and bc[14]>1.5 and abs(wz)<0.12:
        if t-lastopen['CORR']>6:
            lastopen['CORR']=t
            log(f'!!! CORRIDOR hd={hd:.0f} aL={avgL:.2f} aR={avgR:.2f} v={sp:.2f} pose=({px:.1f},{py:.1f})')
            SC.write(f'[{time.time()%100000:.1f}] hd={hd:.0f} pose=({px:.1f},{py:.1f})\n')
    if t<turn_until:
        fl='R'
        try: fl=open('/bot/src/FLAG').read().strip()
        except: pass
        emit(22, 85 if fl=='L' else -85); mode='TURNIN'; continue
    if t<rev_until:
        yaw=1 if FL>FR+0.2 else (-1 if FR>FL+0.2 else 0)
        emit(-45,-yaw*70); mode='REV'; d7c=-yaw*70; continue
    steer=85*(avgL-avgR)+45*(FL-FR)-30*wz
    d7c=max(-85,min(85,steer))
    if front<0.45:
        rev_until=t+0.9; emit(-45,0); mode='TOREV'; continue
    flag=None
    try: flag=open('/bot/src/FLAG').read().strip()
    except: pass
    if flag in ('L','R') and t>5:
        if flag=='L' and avgL>1.5 and t-lastopen['L']>4:
            lastopen['L']=t; turn_until=t+2.2
            log(f'TAKING L hd={hd:.0f} aL={avgL:.2f} pose=({px:.1f},{py:.1f})')
            os.remove('/bot/src/FLAG'); continue
        if flag=='R' and avgR>1.5 and t-lastopen['R']>4:
            lastopen['R']=t; turn_until=t+2.2
            log(f'TAKING R hd={hd:.0f} aR={avgR:.2f} pose=({px:.1f},{py:.1f})')
            os.remove('/bot/src/FLAG'); continue
    aw=abs(wz)
    if aw>0.45 or front<0.9: d4=20; mode='BEND'
    elif aw>0.25 or front<1.4: d4=38; mode='FWD'
    else: d4=75; mode='FAST'
    emit(d4,d7c)
    cyc+=1
    if cyc%80==0: log(f'c{cyc} hd={hd:.0f} v={sp:.2f} pose=({px:.1f},{py:.1f}) th={math.degrees(th)%360:.0f} front={front:.2f} {mode} {d8}')
    time.sleep(0.045)
