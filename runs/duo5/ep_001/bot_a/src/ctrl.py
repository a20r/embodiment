import threading, time, math, os

D='/dev/robot/'
latest={}
TICKS_PER_U=700.0

def fwd_bearing():
    return hdg()%360.0
def hdg():
    v=latest.get('d1')
    try: return float(v[1])
    except: return 0.0

pose=[0.0,0.0]
olock=threading.Lock()
enc={'d7':None,'d8':None}; acc={'d7':0,'d8':0}
def odo_update(p,s):
    try: v=int(float(s))
    except: return
    with olock:
        if enc[p] is None: enc[p]=v; return
        acc[p]+=v-enc[p]; enc[p]=v
        d7,d8=acc['d7'],acc['d8']
        if abs(d7)>=1 and abs(d8)>=1:
            ds=(d7+d8)/2.0/TICKS_PER_U
            acc['d7']=0; acc['d8']=0
            b=math.radians(fwd_bearing())
            pose[0]+=ds*math.cos(b); pose[1]+=ds*math.sin(b)

def reader(p):
    while True:
        try:
            with open(D+p) as f:
                for line in f:
                    s=line.strip()
                    latest[p]=(time.time(),s)
                    if p=='d4' and s:
                        with open('/tmp/radio.log','a') as g: g.write(f"{time.time():.1f} RX {s}\n")
                    if p in ('d7','d8'): odo_update(p,s)
        except Exception:
            time.sleep(0.5)

def get(p):
    v=latest.get(p); return v[1] if v else None
def lidar():
    s=get('d3')
    if not s: return None
    try:
        r=[float(x) for x in s.split(',')]
        return r if len(r)==16 else None
    except: return None
def rel(r,i):  # beam at relative angle i*22.5 clockwise from forward; +4=right, -4=left
    return r[i%16]
def front3(r):
    v=([rel(r,0)] if rel(r,0)>0 else []) + [x+0.12 for x in (rel(r,1),rel(r,-1)) if x>0]
    return min(v) if v else 9.9
def motors(l,r):
    open(D+'d10','w').write(f"{l:.0f}\n"); open(D+'d11','w').write(f"{r:.0f}\n")
def tx(msg):
    open(D+'d0','w').write(msg+"\n")
    with open('/tmp/radio.log','a') as g: g.write(f"{time.time():.1f} TX {msg}\n")
def angdiff(a,b):
    d=(a-b)%360
    return d-360 if d>180 else d

stop_flag=False
def flog(m):
    with open('/tmp/follow.log','a') as f: f.write(f"{time.time():.1f} {m}\n")
def turn_to_fwd(target,tol=5):
    # rotate so forward bearing = target. clockwise(+) = motors(+,-) raises heading
    t0=time.time()
    while not stop_flag and time.time()-t0<25:
        e=angdiff(target,fwd_bearing())
        if abs(e)<tol: break
        sp=max(8,min(45,abs(e)*0.8))
        if e>0: motors(sp,-sp)
        else: motors(-sp,sp)
        time.sleep(0.05)
    motors(0,0)

def fwd(dist,sp=60,frontstop=0.36):
    b0=fwd_bearing()
    with olock: sx,sy=pose[0],pose[1]
    t0=time.time()
    while not stop_flag and time.time()-t0<dist/(sp*0.0012)+8:
        with olock: gone=math.hypot(pose[0]-sx,pose[1]-sy)
        if gone>=dist: break
        r=lidar()
        if r and front3(r)<frontstop: break
        if get('d9')=='1':
            motors(-40,-40); time.sleep(0.5); break
        e=angdiff(b0,fwd_bearing())
        c=max(-15,min(15,e*0.8))
        motors(sp+c,sp-c)
        time.sleep(0.05)
    motors(0,0)

def follow(side='left',secs=9999,sp=45,want=0.22):
    sgn = 1 if side=='right' else -1
    t0=time.time()
    import collections
    hist=collections.deque(maxlen=100)
    while not stop_flag and time.time()-t0<secs:
        r=lidar()
        if not r: time.sleep(0.05); continue
        with olock: hist.append((time.time(),pose[0],pose[1]))
        # stuck detection
        if len(hist)==100 and hist[-1][0]-hist[0][0]>5:
            d=math.hypot(hist[-1][1]-hist[0][1],hist[-1][2]-hist[0][2])
            if d<0.10:
                flog('stuck')
                motors(-50,-50); time.sleep(1.2); motors(0,0)
                turn_to_fwd((fwd_bearing()-sgn*45)%360)
                hist.clear(); continue
        f=front3(r)
        s90=rel(r,4*sgn); s45=rel(r,2*sgn)
        if get('d9')=='1':
            flog('bump')
            motors(-45,-45); time.sleep(0.8); motors(0,0)
            turn_to_fwd((fwd_bearing()-sgn*70)%360); hist.clear(); continue
        if f<0.34:
            flog(f'blocked f={f:.2f}')
            turn_to_fwd((fwd_bearing()-sgn*90)%360)
            # commit: creep forward briefly
            t1=time.time()
            while time.time()-t1<1.0 and not stop_flag:
                if get('d9')=='1': break
                rr=lidar()
                if rr and front3(rr)<0.30: break
                motors(45,45); time.sleep(0.05)
            motors(0,0)
            hist.clear(); continue
        if 0<s90<0.12 or 0<rel(r,3*sgn)<0.12:
            # scraping: rotate away in place
            flog('scrape')
            e=18
            motors(-sgn*35*-1*0+ (35 if sgn==-1 else -35), (-35 if sgn==-1 else 35))
            time.sleep(0.35); motors(0,0); continue
        err=0.0; n=0
        if 0<s90<1.0: err+=(s90-want); n+=1
        if 0<s45<1.2: err+=0.7*(s45-want*1.35); n+=1
        if n==0:
            motors(sp+sgn*14, sp-sgn*14)
        else:
            c=max(-22,min(22,err*80))
            motors(sp+sgn*c, sp-sgn*c)
        time.sleep(0.06)
    motors(0,0)


S=0.55
def walls_here():
    # returns dict bearing->open(True/False) for 0,90,180,270 using raw beams relative to current fwd
    r=lidar()
    if not r: return None
    h=fwd_bearing()
    out={}
    for b in (0,90,180,270):
        # beam index closest to bearing b: rel angle = (b-h) ; index = rel/22.5
        relang=(b-h)%360
        idx=int(round(relang/22.5))%16
        v=r[idx]
        out[b]=9.9 if v<0 else v
    return out

def recenter():
    # use walls to snap to cell center: adjust along fwd axis using front or back wall
    r=lidar()
    if not r: return
    f=rel(r,0); bk=rel(r,8)
    # desired distance to nearest wall along axis = k*S+0.275; residual = f - snap
    def resid(d):
        k=round((d-0.275)/S)
        return d-(k*S+0.275)
    dv=None
    if 0<f<1.2: dv=resid(f)     # dv>0: too far from front wall -> move fwd dv
    elif 0<bk<1.2: dv=-resid(bk)
    if dv is not None and 0.04<abs(dv)<0.25:
        sp=40 if dv>0 else -40
        t=abs(dv)/0.30
        motors(sp,sp); time.sleep(t); motors(0,0)

def move_cell_once(b):
    # face bearing b, drive S, return True if success
    turn_to_fwd(b, tol=4)
    time.sleep(0.15)
    turn_to_fwd(b, tol=4)
    r=lidar()
    if r and 0<rel(r,0)<0.45:
        recenter(); time.sleep(0.2)
        turn_to_fwd(b, tol=4)
        r=lidar()
        if r and 0<rel(r,0)<0.42: return False
    d0=rel(r,0)
    if d0<0: d0=3.0
    target=d0-S
    k=round((target-0.275)/S)
    snap=k*S+0.275
    if abs(snap-target)<0.18 and snap>0.15: target=snap
    t0=time.time()
    ok=True
    with olock: sx,sy=pose[0],pose[1]
    while not stop_flag and time.time()-t0<6:
        with olock: gone=math.hypot(pose[0]-sx,pose[1]-sy)
        if gone>S*1.05 and (target<0.2 or target>1.5): break
        rr=lidar()
        if not rr: time.sleep(0.04); continue
        f=rel(rr,0)
        if f<0: f=9.9
        if get('d9')=='1':
            motors(-45,-45); time.sleep(0.6); motors(0,0); ok=False; break
        if f<=max(target,0.24): break
        # heading hold + lateral centering with side beams
        e=angdiff(b,fwd_bearing())
        c=e*1.2
        L=rel(rr,-4); R=rel(rr,4)
        if 0<L<0.5 and 0<R<0.5: c-= (L-R)*30   # steer toward larger side... sign: if L>R too far right? L big -> need move left=ccw=negative c? 
        elif 0<L<0.20: c+=8
        elif 0<R<0.20: c-=8
        c=max(-18,min(18,c))
        sp=55 if f>target+0.35 else 35
        motors(sp+c,sp-c)
        time.sleep(0.05)
    motors(0,0)
    return ok


def move_cell(b):
    if move_cell_once(b): return True
    recenter(); time.sleep(0.2)
    return move_cell_once(b)

def explore(secs=9999):
    t0=time.time()
    # grid bookkeeping in units of S, axes: x=east(90?) arbitrarly: use bearing 0 as +X, 90 as +Y
    cur=(0,0)
    fails={}
    visited={cur}
    stack=[cur]
    parentdir={}
    DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}
    BACK={0:180,90:270,180:0,270:90}
    path_from={cur:[]}
    done_tries=0
    while not stop_flag and time.time()-t0<secs:
        if goalflag()=='1':
            flog('GOAL REACHED'); motors(0,0)
            with open('/tmp/grid.log','a') as g: g.write(f"GOAL at {cur}\n")
            break
        w=walls_here()
        if not w: time.sleep(0.1); continue
        moved=False
        for b in (0,90,180,270):
            d=DIRS[b]; nxt=(cur[0]+d[0],cur[1]+d[1])
            if nxt in visited or fails.get((cur,b),0)>=3: continue
            if w[b]>0.50:
                flog(f'try {cur}->{nxt} b={b} d={w[b]:.2f}')
                recenter()
                if move_cell(b):
                    visited.add(nxt); parentdir[nxt]=BACK[b]; cur=nxt; moved=True
                    gridpos[0],gridpos[1]=cur
                    with open('/tmp/grid.log','a') as g: g.write(f"{time.time():.0f} cell {cur} n={len(visited)}\n")
                    break
                else:
                    fails[(cur,b)]=fails.get((cur,b),0)+1
                    recenter()
        if not moved:
            if cur==(0,0) and all((cur[0]+dx,cur[1]+dy) in visited for dx,dy in DIRS.values()):
                pass
            btf=getattr(explore,'_btf',0)
            pb=parentdir.get(cur)
            if pb is not None and btf>=4:
                flog('re-rooting after repeated backtrack fails')
                visited.clear(); visited.add(cur); parentdir.clear(); fails.clear()
                explore._btf=0; recenter(); continue
            if pb is None:
                done_tries+=1
                flog(f'rooted, retry {done_tries}')
                if done_tries>6: break
                recenter(); time.sleep(0.5)
                # unpoison: clear fail counts to retry blocked branches
                fails.clear()
                if done_tries>2:
                    visited.clear(); visited.add(cur); parentdir.clear()
                continue
            if move_cell(pb):
                d=DIRS[pb]; cur=(cur[0]+d[0],cur[1]+d[1])
                gridpos[0],gridpos[1]=cur
                explore._btf=0
            else:
                flog(f'backtrack fail at {cur}')
                explore._btf=getattr(explore,'_btf',0)+1
                recenter()
                r=lidar()
                if r and rel(r,0)>0.40: motors(40,40)
                else: motors(-40,-40)
                time.sleep(0.5); motors(0,0)
                turn_to_fwd((fwd_bearing()+30)%360)
    motors(0,0)


def measure_d5(n=8):
    motors(0,0)
    # wait for stabilization (motion inflates d5, decays slowly)
    prev=None; t0=time.time()
    while time.time()-t0<7:
        try: v=float(get('d5'))
        except: v=None
        if v is not None and prev is not None and abs(v-prev)<0.012:
            break
        prev=v; time.sleep(0.5)
    vals=[]
    for _ in range(n):
        try: vals.append(float(get('d5')))
        except: pass
        time.sleep(0.22)
    vals.sort()
    return vals[len(vals)//2] if vals else 0.0

def climb(secs=9999):
    import random
    t0=time.time()
    BACK={0:180,90:270,180:0,270:90}
    lastdir=None; prev=None
    while not stop_flag and time.time()-t0<secs:
        if goalflag()=='1':
            flog('GOAL REACHED (climb)'); motors(0,0)
            with open('/tmp/grid.log','a') as g: g.write("GOAL(climb)\n")
            for _ in range(100):
                tx("B goal 1 here"); time.sleep(3)
            break
        base=measure_d5()
        tx(f"B climb d5 {base:.3f} goal {goalflag()}")
        with open('/tmp/grid.log','a') as g: g.write(f"{time.time():.0f} climb base={base:.3f} last={lastdir}\n")
        w=walls_here()
        if not w: time.sleep(0.2); continue
        dirs=[x for x in (0,90,180,270) if w[x]>0.50]
        if not dirs:
            recenter(); continue
        if prev is not None and lastdir is not None and base>prev+0.005 and lastdir in dirs:
            choice=lastdir
        elif prev is not None and lastdir is not None and base<prev-0.005:
            # got worse: prefer back, else perpendicular
            cand=[BACK[lastdir]]+[d for d in dirs if d!=lastdir and d!=BACK[lastdir]]
            cand=[d for d in cand if d in dirs] or dirs
            choice=cand[0] if random.random()<0.7 else random.choice(cand)
        else:
            # neutral: prefer continuing, else random
            if lastdir in dirs and random.random()<0.5: choice=lastdir
            else: choice=random.choice(dirs)
        prev=base
        if move_cell(choice):
            lastdir=choice
        else:
            lastdir=None  # blocked, try again fresh
    motors(0,0)

def spin_at_goal():
    motors(0,0)
    i=0
    while not stop_flag:
        tx("B goal 1 parked at goal, spinning. home on my motor sound via d5")
        # gentle spin to make noise but stay in cell
        motors(30,-30); time.sleep(1.2); motors(-30,30); time.sleep(1.2)
        motors(0,0); time.sleep(1.5)
        i+=1

def heard_goal():
    try:
        for line in open('/tmp/radio.log').read().splitlines()[-30:]:
            if 'RX' in line and 'goal 1' in line: return True
    except: pass
    return False


def gofar(bearing=0, secs=300):
    # push persistently toward compass bearing; simple anti-oscillation memory
    t0=time.time()
    BACK={0:180,90:270,180:0,270:90}
    recent=[]
    cur=(0,0); DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}
    deadends={}
    lastb=None
    while not stop_flag and time.time()-t0<secs:
        if goalflag()=='1':
            flog('GOAL (gofar)'); motors(0,0)
            with open('/tmp/grid.log','a') as g: g.write("GOAL(gofar)\n")
            spin_at_goal(); return
        w=walls_here()
        if not w: time.sleep(0.2); continue
        opens=[b for b in (0,90,180,270) if w[b]>0.50]
        flog(f"gofar@{cur} w={ {k:round(v,2) for k,v in w.items()} } opens={opens} lastb={lastb}")
        if not opens:
            recenter(); time.sleep(0.3); continue
        # order: toward bearing first, then sides, back last
        rot=[(bearing)%360,(bearing+90)%360,(bearing-90)%360,(bearing+180)%360]
        cand=[b for b in rot if b in opens]
        # avoid immediate backtrack unless forced; avoid recent cells
        best=None
        for b in cand:
            d=DIRS[b]; nxt=(cur[0]+d[0],cur[1]+d[1])
            if deadends.get(nxt,0)>=3: continue
            if lastb is not None and b==BACK[lastb] and len(cand)>1: continue
            if nxt in recent[-6:] and len(cand)>1: continue
            best=b; break
        if best is None: best=cand[0] if cand else opens[0]
        if move_cell(best):
            d=DIRS[best]; cur=(cur[0]+d[0],cur[1]+d[1])
            gridpos[0],gridpos[1]=cur
            recent.append(cur); recent[:-30]=[]
            if len([b for b in (0,90,180,270) if b in opens])==1 and lastb is not None:
                pass
            lastb=best
            with open('/tmp/grid.log','a') as g: g.write(f"{time.time():.0f} gofar {cur} b={best}\n")
        else:
            d=DIRS[best]; k=(cur[0]+d[0],cur[1]+d[1]); deadends[k]=deadends.get(k,0)+1
    motors(0,0)

def brain():
    time.sleep(2)
    tx("B proto: whoever finds goal parks there and spins; other homes on sound (d5 climb). goal flag not found yet")
    while not stop_flag:
        if goalflag()=='1':
            flog('BRAIN: at goal, parking+spinning')
            spin_at_goal(); return
        if heard_goal():
            flog('BRAIN: heard goal 1, climbing sound')
            climb(240)
        else:
            import random
            b=random.choice((0,90,180,270))
            tx(f"B sweeping bearing {b} far")
            gofar(b, 240)
            if goalflag()=='1' or heard_goal(): continue
            explore(180)
        time.sleep(0.5)

def status():
    with olock: x,y=pose
    return f"t={time.time():.0f} pose=({x:.2f},{y:.2f}) fwd={fwd_bearing():.0f} d2={get('d2')} d5={get('d5')} d6={get('d6')} d9={get('d9')} lidar={get('d3')}"

for p in ['d1','d2','d3','d4','d5','d6','d7','d8','d9']:
    threading.Thread(target=reader,args=(p,),daemon=True).start()
gridpos=[0,0]
def goalflag():
    s=get('d6') or ''
    return '1' if 'goal=1' in s else '0'
def beacon():
    while True:
        tx(f"B pos {gridpos[0]} {gridpos[1]} goal {goalflag()}")
        time.sleep(3)
threading.Thread(target=beacon,daemon=True).start()
def telem():
    while True:
        s=status()
        with open('/tmp/telem.log','a') as f: f.write(s+"\n")
        with open('/tmp/state.txt','w') as f: f.write(s+"\n")
        time.sleep(1)
threading.Thread(target=telem,daemon=True).start()

os.path.exists('/tmp/cmd') or os.mkfifo('/tmp/cmd')
cur=None
def runcmd(line):
    parts=line.split()
    if not parts: return
    c=parts[0]
    try:
        if c=='mot': motors(float(parts[1]),float(parts[2])); return
        elif c=='turnto': turn_to_fwd(float(parts[1]))
        elif c=='fwd': fwd(*[float(x) for x in parts[1:]])
        elif c=='follow': follow(parts[1], *[float(x) for x in parts[2:]])
        elif c=='tx': tx(' '.join(parts[1:]))
        elif c=='explore': explore(*[float(x) for x in parts[1:]])
        elif c=='climb': climb(*[float(x) for x in parts[1:]])
        elif c=='brain': brain()
        elif c=='gofar': gofar(*[float(x) for x in parts[1:]])
        elif c=='setpose':
            with olock: pose[0]=float(parts[1]); pose[1]=float(parts[2])
    except Exception as e:
        with open('/tmp/telem.log','a') as f: f.write(f"CMDERR {line}: {e}\n")

while True:
    with open('/tmp/cmd') as f:
        for line in f:
            line=line.strip()
            global_stop = (line=='stop')
            if global_stop:
                stop_flag=True
                time.sleep(0.3)
                if cur: cur.join(timeout=5)
                stop_flag=False; motors(0,0); cur=None
            elif line:
                if cur and cur.is_alive(): continue
                cur=threading.Thread(target=runcmd,args=(line,),daemon=True); cur.start()
