import os, sys, time, json, select, re, signal, math

R='/dev/robot/'
READ_PORTS=['d3','d4','d6','d9','d11','d0','d5','d10']
state={p:None for p in READ_PORTS}
state['d3_goal']=0; state['d3_here']=0
logf=open('/memory/pilot.log','a',buffering=1)
def log(m): logf.write(f"[{time.strftime('%H:%M:%S')}] {m}\n")

def read1(p,dur=0.05):
    try: fd=os.open(R+p,os.O_RDONLY|os.O_NONBLOCK)
    except OSError: return None
    acc=''; t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select([fd],[],[],dur/3)
        if r:
            try:
                d=os.read(fd,4096).decode()
                if d: acc+=d
            except BlockingIOError: pass
    os.close(fd)
    return acc

def lastline(txt):
    ls=[x for x in txt.split('\n') if x.strip()]
    return ls[-1] if ls else None

def write_motors(l,r):
    for p,v in (('d7',l),('d1',r)):
        fd=os.open(R+p,os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd,(str(v)+"\n").encode()); os.close(fd)
def stop(): write_motors(0,0)
def norm180(a,b): return (b-a+540)%360-180
def geth(): return float(last_of4('d4'))
def last_of4(p,dur=0.12):
    for _ in range(3):
        ln=lastline(read1(p,dur) or '')
        if ln:
            try: return float(ln)
            except: pass
    return None

def grab_scan(dur=1.0):
    buf=''
    fd=os.open(R+'d2',os.O_RDONLY|os.O_NONBLOCK)
    t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select([fd],[],[],0.02)
        if r:
            try: buf+=os.read(fd,4096).decode()
            except BlockingIOError: pass
    os.close(fd)
    pts=[]
    for x in buf.replace('\n',';').split(';'):
        m=re.findall(r'-?\d*\.?\d+',x)
        if len(m)==3:
            try: pts.append(tuple(map(float,m)))
            except ValueError: pass
    # valid returns only
    return [(abs(a),b,c) for a,b,c in pts if abs(a)>0.15]

def clearance_grid(pts, nbins=9):
    # az bins across -0.25..0.25
    edges=[-0.25+i*0.5/nbins for i in range(nbins+1)]
    grid=[[] for _ in range(nbins)]
    for r,c1,c2 in pts:
        if c1<-0.15 or c1>0.15: continue  # body-height band only
        for i in range(nbins):
            if edges[i]<=c2<edges[i+1]:
                grid[i].append(r); break
    med=[]
    for g in grid:
        med.append(round(sorted(g)[len(g)//2],3) if len(g)>=5 else None)
    return med

def do_scan():
    pts=grab_scan(1.0)
    g=clearance_grid(pts)
    return {"grid_az_-0.25..0.25":g}

def check_goal():
    ln=lastline(read1('d3',0.06) or '')
    if ln and 'goal=1' in ln:
        log("!!! GOAL=1 !!! "+ln); return True
    return False

def do_rot(deg,cmd=50):
    # single-wheel pivot: CW (+deg): d1=+cmd (d9/LEFT wheel); CCW: d7=+cmd (d6/RIGHT wheel)
    h0=geth()
    if h0 is None: return {"err":"no heading"}
    tgt=h0+deg
    port='d1' if deg>0 else 'd7'
    rate=0.95*cmd  # deg/s approx
    tmax=abs(deg)/rate+2.0
    fd=os.open(R+port,os.O_WRONLY|os.O_NONBLOCK)
    os.write(fd,(str(cmd)+"\n").encode()); os.close(fd)
    t0=time.time(); broke=False
    while time.time()-t0<tmax:
        h=geth()
        if h is not None and abs(norm180(tgt,h))<4: broke=True; break
        if check_goal(): stop(); log("GOAL during rot"); break
    stop(); time.sleep(0.15)
    h1=geth()
    return {"rot":round(norm180(h0,h1),1) if (h0 is not None and h1 is not None) else None,"broke":broke,"t":round(time.time()-t0,2)}

def do_fwd(sec,cmd=40):
    e0l=lastline(read1('d6',0.1)); e0r=lastline(read1('d9',0.1))
    r0=None
    pts=grab_scan(0.5)
    if pts:
        front=[r for r,c1,c2 in pts if abs(c2)<0.09 and c1>-0.3]
        if front: r0=sorted(front)[len(front)//2]
    t0=time.time(); write_motors(cmd,cmd)
    while time.time()-t0<sec:
        if check_goal(): log("GOAL during fwd"); break
    stop()
    e1l=lastline(read1('d6',0.1)); e1r=lastline(read1('d9',0.1))
    try: dl=int(e1l)-int(e0l); dr=int(e1r)-int(e0r)
    except: dl=dr=-1
    pts=grab_scan(0.5)
    r1=None
    if pts:
        front=[r for r,c1,c2 in pts if abs(c2)<0.09 and c1>-0.3]
        if front: r1=sorted(front)[len(front)//2]
    tpm=None
    if r0 is not None and r1 is not None and r0>r1+0.05:
        tpm=round(((dl+dr)/2)/(r0-r1),1)
    return {"ticks_l":dl,"ticks_r":dr,"front_r0":r0,"front_r1":r1,"ticks_per_m":tpm}

def do_tx(msg):
    try:
        fd=os.open(R+'d8',os.O_WRONLY|os.O_NONBLOCK)
        n=os.write(fd,(msg+"\n").encode()); os.close(fd)
        return {"tx":msg}
    except Exception as e: return {"txerr":str(e)}

def do_probe11():
    out=[]
    h0=geth()
    for i in range(12):
        d11=lastline(read1('d11',0.12))
        out.append({"h":round((geth() or 0),0),"d11":d11})
        do_rot(30,cmd=40)
    return out

def do_explore(n=25):
    results=[]
    tpm_est=[]
    for it in range(n):
        if check_goal(): log("GOAL=1 in explore!"); break
        pts=grab_scan(1.0)
        g=clearance_grid(pts)
        front=g[len(g)//2]
        h=geth()
        res={"it":it,"h":round(h,0) if h is not None else None,"grid":g}
        if front is not None and front>0.36:
            f=do_fwd(1.4,40)
            res["fwd"]=f
            if f.get("ticks_per_m"): tpm_est.append(f["ticks_per_m"])
        else:
            # pick side with more open space: bins 0-3 = az -0.25..0, bins 5-8 = az 0..0.25
            def sc(bins):
                vals=[g[i] for i in bins if g[i] is not None]
                return sum(vals)/len(vals) if vals else 0
            left=sc(range(0,4)); right=sc(range(5,9))
            deg = -45 if left>right else 45
            res["turn"]=deg; res["lr"]=[round(left,2),round(right,2)]
            do_rot(deg,50)
        results.append(res)
        if it%5==0:
            do_tx(f"PING ep1 {it}")
    json.dump(results,open('/memory/explore_last.json','w'))
    avg=round(sum(tpm_est)/len(tpm_est),1) if tpm_est else None
    log("explore done, ticks_per_m est: "+str(avg))
    return {"done":True,"ticks_per_m":avg,"last_grid":g}


def do_panscan():
    out=[]
    for i in range(8):
        pts=grab_scan(0.8)
        g=clearance_grid(pts)
        out.append({"h":round(geth() or -1,0),"grid":g})
        if i<7: do_rot(45,cmd=60)
    json.dump(out,open('/memory/panscan.json','w'))
    close=[]
    for e in out:
        vals=[v for v in e["grid"] if v is not None]
        if vals: close.append((e["h"],round(min(vals),2)))
    return {"min_per_heading":close}

def do_txburst(sec=20):
    t0=time.time(); n=0
    while time.time()-t0<sec:
        do_tx(f"PING {n} tick={state.get('tick')}")
        n+=1; time.sleep(1.0)
    return {"sent":n,"rx":state['d10']}

def handle(op):
    t=op.get('op')
    if t=='stop': stop(); return {"ok":1}
    if t=='vel': write_motors(op.get('l',0),op.get('r',0)); return {"ok":1}
    if t=='rot': return do_rot(float(op.get('deg',0)),int(op.get('cmd',50)))
    if t=='fwd': return do_fwd(float(op.get('sec',1)),int(op.get('cmd',40)))
    if t=='scan': return do_scan()
    if t=='tx': return do_tx(str(op.get('msg','')))
    if t=='rx': return {"rx":state['d10']}
    if t=='sense': return {k:state[k] for k in ['d3','d4','d6','d9','d11','d0','d5','d10']}
    if t=='probe11': return do_probe11()
    if t=='explore': return do_explore(int(op.get('n',25)))
    if t=='panscan': return do_panscan()
    if t=='txburst': return do_txburst(int(op.get('sec',20)))
    return {"err":"unknown op "+str(t)}

running=True
def bye(sig,f):
    global running; running=False; stop(); log("SIGTERM -> stopped")
signal.signal(signal.SIGTERM,bye); signal.signal(signal.SIGINT,bye)

open('/memory/cmd.txt','w').close()
log("pilot v2 started pid "+str(os.getpid()))
i=0; lastgoal=None; lasthere=None
while running:
    p=READ_PORTS[i%len(READ_PORTS)]
    txt=read1(p,0.04)
    ln=lastline(txt) if txt else None
    if ln:
        state[p]=ln
        if p=='d3':
            try:
                d=dict(kv.split('=') for kv in ln.split() if '=' in kv)
                g_,h_=int(d.get('goal',-1)),int(d.get('here',-1))
                state['d3_goal'],state['d3_here']=g_,h_; state['tick']=int(d.get('tick',-1))
                if lastgoal is not None and g_!=lastgoal: log(f"GOAL FLAG CHANGE: goal={g_} here={h_}")
                if lasthere is not None and h_!=lasthere: log(f"HERE FLAG CHANGE: goal={g_} here={h_}")
                lastgoal,lasthere=g_,h_
            except Exception: pass
        if p=='d10' and ln.strip():
            log("RADIO RX: "+ln.strip()); open('/memory/rx.log','a').write(ln.strip()+'\n')
        if p in ('d0','d5') and ln.strip() not in ('0',''):
            log(f"{p} nonzero: {ln.strip()}")
    i+=1
    try:
        lines=open('/memory/cmd.txt').read().strip().split('\n')
        if lines and lines[0]:
            open('/memory/cmd.txt','w').close()
            for L in lines:
                try:
                    op=json.loads(L); res=handle(op)
                    log("CMD "+L[:120]+" -> "+json.dumps(res)[:600])
                except Exception as e:
                    log("CMD ERR "+L[:80]+" : "+repr(e)); stop()
    except Exception as e: log("cmdfile err "+repr(e))
    if i%80==0: json.dump(state,open('/memory/state.json','w'))
stop(); log("pilot exited")
