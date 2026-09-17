import time, math, os, threading, select, re, traceback
NB=os.O_NONBLOCK
P={p:os.open(f'/dev/robot/{p}', os.O_RDONLY|NB) for p in ('d0','d2','d5','d8')}
latest={p:'' for p in P}
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
        except Exception: pass
for p in P: threading.Thread(target=reader,args=(p,),daemon=True).start()
def get(p):
    with lock: return latest[p]
def f(x,d=0.0):
    try: return float(x)
    except: return d
LOG=open('/bot/src/mapmon.log','a',buffering=1)
def log(m): LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
D8RE=re.compile(r'tick=(\d+) goal=(-?\d+) lap=(-?\d+) last=([-\d.]+) best=([-\d.]+)')
FIRELOG=open('/bot/src/FIRELOG.log','a',buffering=1)
# frame init from ap9 csv tail
px=py=0.0; th=0.0
try:
    last=None
    for l in open('/bot/src/auto9.csv'): last=l
    a=last.split(',')
    px,py,th=float(a[2]),float(a[3]),math.radians(float(a[4]))
    log(f'frame init from ap9: ({px:.2f},{py:.2f}) th={math.degrees(th):.1f}')
except Exception as e:
    log('frame init fail '+str(e))
wzbias=0.027
# grid
MS=400; RES=0.2; OFF=MS//2
grid=[[0]*MS for _ in range(MS)]
def raycast(rng):
    n=int(rng/RES)
    cx,cy=math.cos(th),math.sin(th)
    for i in range(16):
        a=th+i*math.pi/8
        ca,sa=math.cos(a),math.sin(a)
        r=bcl[i]
        if r<0: continue
        r=min(r,5.9)
        k=int(r/RES)
        for q in range(1,k):
            x=px+ca*q*RES; y=py+sa*q*RES
            gi=int(y/RES)+OFF; gj=int(x/RES)+OFF
            if 0<=gi<MS and 0<=gj<MS:
                if grid[gi][gj]==0: grid[gi][gj]=1
            else: break
        gi=int((py+sa*r)/RES)+OFF; gj=int((px+ca*r)/RES)+OFF
        if 0<=gi<MS and 0<=gj<MS: grid[gi][gj]=2
prev={'lap':None}
lastt=None; hb=0; mapt=0; cyc=0
log('mapmon start')
while True:
    try:
        s=get('d5')
        try: b=[float(x) for x in s.split(',')]
        except Exception: time.sleep(0.05); continue
        if len(b)!=16: time.sleep(0.05); continue
        bcl=b
        sp=f(get('d2'))
        try: wz=f(get('d0').split(',')[2])
        except Exception: wz=0.0
        t=time.time()
        if lastt is None: lastt=t
        dt=min(t-lastt,0.25); lastt=t
        if abs(sp)<0.02 and abs(wz)<0.15: wzbias+=0.02*(wz-wzbias)
        wzc=wz-wzbias
        th=(th+wzc*dt)%(2*math.pi)
        px+=sp*math.cos(th)*dt; py+=sp*math.sin(th)*dt
        raycast(0)  # full sweep using current bcl
        m=D8RE.search(get('d8'))
        if m:
            lap=m.group(3)
            if prev['lap'] is not None and lap!=prev['lap']:
                FIRELOG.write(f'[{t%100000:8.1f}] lap->{lap} pose=({px:.2f},{py:.2f}) th={math.degrees(th)%360:.1f} tick={m.group(1)}\n')
                with open('/bot/src/ZONE','w') as z: z.write(f'{px:.2f} {py:.2f}\n')
                log(f'FIRE lap->{lap} pose=({px:.2f},{py:.2f})')
            prev['lap']=lap
        with open('/bot/src/POSEM','w') as p2: p2.write(f'{px:.3f} {py:.3f} {math.degrees(th)%360:.3f}\n')
        if t-mapt>20:
            mapt=t
            with open('/bot/src/mapm.txt','w') as mm:
                for i in range(MS-1,-1,-2):
                    row=''
                    for j in range(0,MS,2):
                        v=max(grid[i][j],grid[i][j-1] if j>0 else 0,grid[i-1][j] if i>0 else 0,grid[i-1][j-1] if i>0 and j>0 else 0)
                        row+='#' if v==2 else ('.' if v==1 else ' ')
                    mm.write(row+'\n')
                mm.write(f'pose {px:.1f} {py:.1f} th {math.degrees(th)%360:.1f}\n')
        cyc+=1
        if cyc%600==0: log(f'c{cyc} pose=({px:.1f},{py:.1f}) v={sp:.2f} bias={wzbias:.4f}')
        time.sleep(0.045)
    except Exception:
        try: log('EXC '+traceback.format_exc().replace('\n',' | ')[-200:])
        except Exception: pass
        time.sleep(0.1)
