import os, time, json, select, re, signal, threading, math

R='/dev/robot/'
TPM=2000.0
state={p:None for p in ['d3','d4','d6','d9','d11','d0','d5']}
state['goal']=0; state['here']=0; state['tick']=0
lock=threading.Lock(); pause_poll={'v':False}; running={'v':True}
logf=open('/memory/pilot.log','a',buffering=1)
def log(m): logf.write(f"[{time.strftime('%H:%M:%S')}] {m}\n")
def read1(p,dur=0.05):
    try: fd=os.open(R+p,os.O_RDONLY|os.O_NONBLOCK)
    except OSError: return None
    acc=''; t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select([fd],[],[],0.02)
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
def poller():
    i=0
    ps=['d4','d4','d3','d11','d6','d9','d0','d5']
    while running['v']:
        if pause_poll['v']: time.sleep(0.05); continue
        p=ps[i%len(ps)]
        ln=lastline(read1(p,0.05) or '')
        if ln:
            with lock:
                state[p]=ln
                if p=='d3':
                    try:
                        d=dict(kv.split('=') for kv in ln.split() if '=' in kv)
                        state['goal']=int(d.get('goal',-1)); state['here']=int(d.get('here',-1)); state['tick']=int(d.get('tick',-1))
                    except Exception: pass
            if p in ('d0','d5') and ln.strip()=='1': log(f"FLAG {p}=1")
        i+=1
        if i%20==0:
            try:
                with lock: snap=dict(state)
                json.dump(snap,open('/memory/state.json','w'))
            except Exception: pass
def radio_thread():
    while running['v']:
        try:
            fd=os.open(R+'d10',os.O_RDONLY|os.O_NONBLOCK)
            t0=time.time()
            while running['v'] and time.time()-t0<0.4:
                r,_,_=select.select([fd],[],[],0.05)
                if r:
                    try:
                        d=os.read(fd,4096).decode().strip()
                        if d: log("RADIO RX: "+d); open('/memory/rx.log','a').write(d+"\n")
                    except Exception: pass
            os.close(fd)
        except Exception: time.sleep(0.2)
def write_motors(l,r):
    for p,v in (('d7',l),('d1',r)):
        fd=os.open(R+p,os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd,(str(v)+"\n").encode()); os.close(fd)
def stop(): write_motors(0,0)
def norm180(a,b): return (b-a+540)%360-180
def geth():
    with lock: v=state['d4']
    try: return float(v)
    except: return None
def goal_seen():
    with lock: return state['goal']==1
def here_seen():
    with lock: return state['here']==1
def enc():
    with lock:
        try: return int(state['d6']),int(state['d9'])
        except: return None,None
def d11avg(n=6):
    vs=[]
    for _ in range(n):
        ln=lastline(read1('d11',0.1) or '')
        try: vs.append(float(ln))
        except: pass
    return sum(vs)/len(vs) if vs else None
def drive_ticks(ticks_l, ticks_r, cmd, timeout=8.0):
    # closed loop on encoders; cmd sign sets direction
    if ticks_l==0 and ticks_r==0: return
    s=1 if (ticks_l+ticks_r)>0 else -1
    c=abs(cmd)*s
    write_motors(c,c)
    t0=time.time()
    e0=enc(); el0,er0=(e0 if e0[0] is not None else (0,0))
    while time.time()-t0<timeout:
        e=enc()
        if e[0] is None: continue
        dl=e[0]-el0; dr=e[1]-er0
        if s>0 and (dl>=ticks_l or dr>=ticks_r): break
        if s<0 and (dl<=ticks_l or dr<=ticks_r): break
        if goal_seen(): log("GOAL during drive_ticks"); break
    stop(); time.sleep(0.1)
def do_rot(deg,cmd=55):
    pause_poll['v']=True
    try:
        h0=geth()
        if h0 is None: return {"err":"noh"}
        tgt=h0+deg
        write_motors(*( (cmd,0) if deg>0 else (0,cmd) ))
        t0=time.time(); broke=False
        while time.time()-t0<abs(deg)/(0.95*cmd)+1.5:
            ln=lastline(read1('d4',0.08) or '')
            try: h=float(ln)
            except: continue
            if abs(norm180(tgt,h))<4: broke=True; break
            if goal_seen(): log("GOAL rot"); break
        stop(); time.sleep(0.15)
        h1=geth()
    finally: pause_poll['v']=False
    return {"rot":round(norm180(h0,h1),1) if h1 is not None else None,"ok":broke}
def scan_clear():
    pause_poll['v']=True
    try:
        buf=''
        fd=os.open(R+'d2',os.O_RDONLY|os.O_NONBLOCK)
        t0=time.time()
        while time.time()-t0<0.7:
            r,_,_=select.select([fd],[],[],0.02)
            if r:
                try: buf+=os.read(fd,4096).decode()
                except BlockingIOError: pass
        os.close(fd)
    finally: pause_poll['v']=False
    pts=[]
    for x in buf.replace('\n',';').split(';'):
        m=re.findall(r'-?\d*\.?\d+',x)
        if len(m)==3:
            try: pts.append(tuple(map(float,m)))
            except ValueError: pass
    pts=[(abs(a),b,c) for a,b,c in pts if abs(a)>0.15]
    edges=[-0.25+i*0.5/9 for i in range(10)]
    grid=[[] for _ in range(9)]
    for r_,c1,c2 in pts:
        if c1<-0.15 or c1>0.15: continue
        for i in range(9):
            if edges[i]<=c2<edges[i+1]: grid[i].append(r_); break
    return [round(sorted(g)[len(g)//2],3) if len(g)>=4 else None for g in grid]
def do_tx(msg):
    try:
        fd=os.open(R+'d8',os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd,(msg+"\n").encode()); os.close(fd)
    except Exception: pass

def do_g2(sec=300):
    # star-search gradient on d11 with odometry returns
    t_end=time.time()+sec
    h=geth(); pos=[0.0,0.0]; hd=h if h is not None else 0.0
    def fwd_ticks(m,cmd=90):
        t=int(m*TPM)
        drive_ticks(t,t,cmd)
    def move(m):
        # update pos by m along heading hd
        nonlocal pos
        rad=math.radians(hd)
        # d4 CW+ ; define x=north-ish: dx=m*cos, dy=m*sin with heading as math angle CW from x-axis
        pos[0]+=m*math.cos(rad); pos[1]+=m*math.sin(rad)
    base=d11avg(6)
    log(f"g2 start d11={base} h={hd}")
    results=[]; hist=[]; sign='min'
    STEP=0.4
    best_dir=None; bestv=None
    it=0
    while time.time()<t_end:
        it+=1
        if goal_seen(): log("GOAL in g2"); break
        if here_seen(): log("HERE in g2"); break
        g=scan_clear(); front=g[4]
        if front is not None and front<0.4:
            # blocked: rotate 70 and continue
            do_rot(70,60); hd=geth() or hd; continue
        v0=d11avg(5)
        do_tx(f"G{it} {round(v0,3) if v0 else ''}")
        trial=[]
        for k,deg in enumerate([0,90,180,-90]):
            if time.time()>t_end: break
            do_rot(deg,60); hd=geth() or hd
            fwd_ticks(STEP); move(STEP); 
            if goal_seen(): log("GOAL probe"); break
            v=d11avg(5)
            trial.append((deg,round(v,3) if v is not None else None,round(hd,1)))
            fwd_ticks(-STEP); move(-STEP); time.sleep(0.3)
        # pick best: try BOTH signs; choose direction with max |v-v0|? no: track sign preference
        valid=[(deg,v,hh) for deg,v,hh in trial if v is not None]
        if not valid: continue
        hist.append(v0)
        if len(hist)>=4:
            a=sum(hist[:len(hist)//2])/max(1,len(hist)//2); b=sum(hist[len(hist)//2:])/(len(hist)-len(hist)//2)
            if sign=='min' and b>a+0.05: sign='max'; log('g2 SIGN FLIP -> max')
            if sign=='max' and b<a-0.05: sign='min'; log('g2 SIGN FLIP -> min')
        if sign=='min': prefs=sorted(valid,key=lambda x:x[1])
        else: prefs=sorted(valid,key=lambda x:-x[1])
        pick=prefs[0]
        best_dir=pick
        do_rot(pick[2]-(geth() or pick[2]),55) if False else None
        # rotate to the picked heading: we returned to base with heading hd_base_before_trials
        # simplest: rotate by (pick_h - current)
        cur=geth() or 0
        delta=norm180(cur,pick[2])
        if abs(delta)>2: do_rot(delta,55)
        # drive 1.0m in picked direction
        fwd_ticks(1.0); move(1.0)
        results.append({"it":it,"v0":round(v0,3) if v0 else None,"trials":trial,"pick":pick})
        log(f"g2 it{it} v0={v0} trials={trial} pick={pick[0]}")
    json.dump(results,open('/memory/g2log.json','w'))
    return {"its":it,"pos":[round(pos[0],2),round(pos[1],2)],"last":results[-1] if results else None}


def do_g3(sec=420):
    t_end=time.time()+sec
    log("g3 start")
    best_min=9; best_max=0; mode='min'
    out=[]
    while time.time()<t_end:
        if goal_seen(): log("GOAL in g3"); break
        if here_seen(): log("HERE in g3"); break
        # rotate 360 sampling d11
        samples=[]
        h0=geth() or 0
        tgt=h0+350
        write_motors(25,0)
        t0=time.time()
        while time.time()-t0<16:
            ln=lastline(read1('d4',0.03) or '')
            try: h=float(ln)
            except: h=None
            d=d11avg(1)
            if h is not None and d is not None: samples.append((h,d))
            if h is not None and abs(norm180(tgt,h))<6: break
            if goal_seen(): break
        stop(); time.sleep(0.2)
        if len(samples)<20: continue
        vals=[d for h,d in samples]
        import statistics as st
        med=st.median(vals)
        vmin=min(samples,key=lambda x:x[1]); vmax=max(samples,key=lambda x:x[1])
        # deviational extreme
        if (med-vmin[1])>(vmax[1]-med): pick,vext,mode2=vmin,vmin[1],'min'
        else: pick,vext,mode2=vmax,vmax[1],'max'
        out.append({"med":round(med,3),"min":round(vmin[1],3),"max":round(vmax[1],3),"pick_h":round(pick[0],1),"mode":mode2})
        do_tx(f"G3 {round(med,2)}")
        log(f"g3 sweep med={med:.3f} min={vmin[1]:.3f}@{vmin[0]:.0f} max={vmax[1]:.3f}@{vmax[0]:.0f} pick={mode2}@{pick[0]:.0f}")
        # rotate to picked heading
        cur=geth() or 0
        delta=norm180(cur,pick[0])
        if abs(delta)>3: do_rot(delta,55)
        # drive toward it if clear
        g=scan_clear(); front=g[4]
        if front is None or front>0.45:
            write_motors(80,80); t0=time.time()
            while time.time()-t0<1.6:
                if goal_seen() or here_seen(): break
            stop()
        else:
            do_rot(60,55)
        time.sleep(0.3)
    json.dump(out,open('/memory/g3log.json','w'))
    return {"sweeps":len(out),"last":out[-1] if out else None}


def do_hunt(sec=900):
    t_end=time.time()+sec
    log("HUNT start (exclusive)")
    out=[]
    it=0
    pause_poll['v']=True
    def upd():
        ln=lastline(read1('d3',0.05) or '')
        if ln:
            try:
                d=dict(kv.split('=') for kv in ln.split() if '=' in kv)
                with lock:
                    state['goal']=int(d.get('goal',-1)); state['here']=int(d.get('here',-1)); state['tick']=int(d.get('tick',-1)); state['d3']=ln
            except Exception: pass
    try:
        while time.time()<t_end:
        it+=1
        if goal_seen(): log("!!! GOAL=1 !!! STOPPING"); stop(); do_tx("A ATGOAL ATGOAL"); break
        if here_seen(): log("!!! HERE=1 !!!"); do_tx("A HERE=1"); break
        upd()
        if state['goal']==1: log("!!! GOAL=1 !!!"); stop(); do_tx("A ATGOAL"); break
        if state['here']==1: log("!!! HERE=1 !!!"); do_tx("A HERE1"); break
        do_tx(f"PING A{it}")
        # 360 d11 sweep
        samples=[]
        h0=geth() or 0
        tgt=h0+350
        write_motors(25,0)
        t0=time.time()
        while time.time()-t0<16:
            ln=lastline(read1('d4',0.03) or '')
            try: h=float(ln)
            except: h=None
            d=d11avg(1)
            if h is not None and d is not None: samples.append((h,d))
            if h is not None and abs(norm180(tgt,h))<6: break
            if goal_seen() or here_seen(): break
        stop(); time.sleep(0.2)
        if len(samples)<15: continue
        vmin=min(samples,key=lambda x:x[1])
        med=sorted(x[1] for x in samples)[len(samples)//2]
        out.append({"it":it,"med":round(med,3),"min":round(vmin[1],3),"minh":round(vmin[0],0)})
        log(f"HUNT it{it} med={med:.3f} min={vmin[1]:.3f}@{vmin[0]:.0f}")
        cur=geth() or 0
        delta=norm180(cur,vmin[0])
        if abs(delta)>3: do_rot(delta,55)
        g=scan_clear()
        front=g[4]
        if it%2==0: json.dump({"grid":g},open('/memory/hunt_scan.json','w'))
        if front is None or front>0.45:
            write_motors(70,70); tA=time.time()
            while time.time()-tA<1.3:
                if goal_seen() or here_seen(): break
            stop()
        else:
            log(f"HUNT blocked front={front} grid={g}")
            do_rot(50,55)
        time.sleep(0.2)
        json.dump(out,open('/memory/huntlog.json','w'))
    finally:
        pause_poll['v']=False
    json.dump(out,open('/memory/huntlog.json','w'))
    return {"its":it,"last":out[-1] if out else None}

def handle(op):
    t=op.get('op')
    if t=='stop': stop(); return {"ok":1}
    if t=='rot': return do_rot(float(op.get('deg',0)),int(op.get('cmd',55)))
    if t=='sense':
        with lock: return {k:state[k] for k in ['d3','d4','d6','d9','d11','d0','d5']}
    if t=='tx': do_tx(str(op.get('msg',''))); return {"ok":1}
    if t=='g2': return do_g2(float(op.get('sec',300)))
    if t=='g3': return do_g3(float(op.get('sec',420)))
    if t=='hunt': return do_hunt(float(op.get('sec',900)))
    return {"err":"unknown "+str(t)}

def bye(sig,f):
    running['v']=False; stop(); log("SIGTERM")
signal.signal(signal.SIGTERM,bye); signal.signal(signal.SIGINT,bye)
open('/memory/cmd.txt','w').close()
threading.Thread(target=poller,daemon=True).start()
threading.Thread(target=radio_thread,daemon=True).start()
log("pilot8 started pid "+str(os.getpid()))
while running['v']:
    try:
        lines=open('/memory/cmd.txt').read().strip().split('\n')
        if lines and lines[0]:
            open('/memory/cmd.txt','w').close()
            for L in lines:
                try:
                    op=json.loads(L); res=handle(op)
                    log("CMD "+L[:100]+" -> "+json.dumps(res)[:400])
                except Exception as e:
                    log("CMD ERR "+L[:80]+": "+repr(e)); stop()
        else: time.sleep(0.1)
    except Exception as e:
        log("loop err "+repr(e)); time.sleep(0.2)
stop(); log("pilot8 exited")
