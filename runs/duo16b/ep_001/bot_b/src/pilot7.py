import os, time, json, select, re, signal, threading

R='/dev/robot/'
state={p:None for p in ['d3','d4','d6','d9','d11','d0','d5']}
state['goal']=0; state['here']=0; state['tick']=0
lock=threading.Lock()      # sensor access
pause_poll={'v':False}
running={'v':True}
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
    while running['v']:
        if pause_poll['v']: time.sleep(0.05); continue
        ps=['d4','d4','d3','d11','d6','d9','d0','d5']
        p=ps[i%len(ps)]
        ln=lastline(read1(p,0.05) or '')
        if ln:
            with lock:
                state[p]=ln
                if p=='d3':
                    try:
                        d=dict(kv.split('=') for kv in ln.split() if '=' in kv)
                        g_,h_=int(d.get('goal',-1)),int(d.get('here',-1))
                        state['goal'],state['here'],state['tick']=g_,h_,int(d.get('tick',-1))
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
                        if d:
                            log("RADIO RX: "+d); open('/memory/rx.log','a').write(d+"\n")
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
    with lock: 
        v=state['d4']
    try: return float(v)
    except: return None
def goal_seen():
    with lock: return state['goal']==1
def here_seen():
    with lock: return state['here']==1

def grab_scan(dur=0.8):
    pause_poll['v']=True
    try:
        buf=''
        fd=os.open(R+'d2',os.O_RDONLY|os.O_NONBLOCK)
        t0=time.time()
        while time.time()-t0<dur:
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
    return [(abs(a),b,c) for a,b,c in pts if abs(a)>0.15]

def clearance_grid(pts, nbins=9):
    edges=[-0.25+i*0.5/nbins for i in range(nbins+1)]
    grid=[[] for _ in range(nbins)]
    for r,c1,c2 in pts:
        if c1<-0.15 or c1>0.15: continue
        for i in range(nbins):
            if edges[i]<=c2<edges[i+1]: grid[i].append(r); break
    return [round(sorted(g)[len(g)//2],3) if len(g)>=4 else None for g in grid]

def do_scan():
    g=clearance_grid(grab_scan(0.8)); return {"grid":g}

def do_rot(deg,cmd=50):
    pause_poll['v']=True
    try:
        h0=geth()
        if h0 is None: return {"err":"noh"}
        tgt=h0+deg; port='d1' if deg>0 else 'd7'
        rate=0.95*cmd; tmax=abs(deg)/rate+1.5
        write_motors(*( (cmd,0) if port=='d1' else (0,cmd) ))
        t0=time.time(); broke=False
        while time.time()-t0<tmax:
            ln=lastline(read1('d4',0.08) or '')
            try: h=float(ln)
            except: continue
            if abs(norm180(tgt,h))<4: broke=True; break
            if goal_seen(): log("GOAL during rot"); break
        stop(); time.sleep(0.15)
        h1=geth()
    finally: pause_poll['v']=False
    return {"rot":round(norm180(h0,h1),1) if h1 is not None else None,"broke":broke}

def d11avg(n=4):
    vs=[]
    for _ in range(n):
        ln=lastline(read1('d11',0.1) or '')
        try: vs.append(float(ln))
        except: pass
    return sum(vs)/len(vs) if vs else None

def do_fwd(sec,cmd=40):
    e0l=lastline(read1('d6',0.1)); e0r=lastline(read1('d9',0.1))
    front=None
    g=clearance_grid(grab_scan(0.5))
    if g[4] is not None: front=g[4]
    t0=time.time(); write_motors(cmd,cmd)
    while time.time()-t0<sec:
        if goal_seen(): log("GOAL during fwd"); break
        if here_seen(): log("HERE during fwd"); break
    stop()
    e1l=lastline(read1('d6',0.1)); e1r=lastline(read1('d9',0.1))
    try: dl=int(e1l)-int(e0l); dr=int(e1r)-int(e0r)
    except: dl=dr=-1
    return {"ticks":(dl,dr),"front":front}

def do_tx(msg):
    try:
        fd=os.open(R+'d8',os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd,(msg+"\n").encode()); os.close(fd); return {"tx":msg}
    except Exception as e: return {"err":str(e)}

def do_explore(sec=60):
    t_end=time.time()+sec; res=[]; it=0
    while time.time()<t_end:
        it+=1
        if goal_seen(): log("!!! GOAL=1 in explore !!!"); break
        g=clearance_grid(grab_scan(0.7))
        front=g[4]
        h=geth()
        r={"it":it,"h":h,"grid":g}
        if front is not None and front>0.5:
            do_tx(f"P{it} h{h}")
            if front>0.9: f=do_fwd(2.6,60)
            else: f=do_fwd(1.5,45)
            r["fwd"]=f
        else:
            l=[v for v in g[1:4] if v is not None]; rt=[v for v in g[5:8] if v is not None]
            ls=sum(l)/len(l) if l else 0; rs=sum(rt)/len(rt) if rt else 0
            if ls>rs: res_deg=-50; write_motors(0,55)
            else: res_deg=50; write_motors(55,0)
            r["turn"]=res_deg
            tA=time.time()
            while time.time()-tA<0.5:
                if goal_seen(): break
            stop()
        res.append(r)
    return {"iters":it,"last":res[-1] if res else None}

def do_gradient(sec=90):
    # amoeba on d11: drive, sample, keep improvements
    t_end=time.time()+sec
    base=d11avg(5)
    best=base; log(f"gradient start d11={base}")
    trail=[]
    dirs=[0,60,-60,120,-120]
    di=0
    while time.time()<t_end:
        if goal_seen(): log("GOAL in gradient"); break
        deg=dirs[di%len(dirs)]; di+=1
        if deg: do_rot(deg,55)
        f=do_fwd(0.9,50)
        v=d11avg(5)
        trail.append({"h":geth(),"d11":v,"ticks":f["ticks"]})
        log(f"grad: rot{deg} d11={v}")
        if v is not None and best is not None and v<best-0.03:
            best=v; di=0   # keep going this way: reset direction cycle to 0 (straight)
            dirs2=dirs; dirs=[0]+dirs2[1:]
        elif v is not None and best is not None and v>best+0.15:
            # worse: undo by 180 later; just continue cycle
            pass
    return {"best":best,"last":trail[-3:]}

def handle(op,t0):
    t=op.get('op')
    if t=='stop': stop(); return {"ok":1}
    if t=='vel': write_motors(op.get('l',0),op.get('r',0)); return {"ok":1}
    if t=='rot': return do_rot(float(op.get('deg',0)),int(op.get('cmd',50)))
    if t=='fwd': return do_fwd(float(op.get('sec',1)),int(op.get('cmd',40)))
    if t=='scan': return do_scan()
    if t=='tx': return do_tx(str(op.get('msg','')))
    if t=='sense':
        with lock: return {k:state[k] for k in ['d3','d4','d6','d9','d11','d0','d5']}
    if t=='explore': return do_explore(float(op.get('sec',60)))
    if t=='gradient': return do_gradient(float(op.get('sec',90)))
    return {"err":"unknown "+str(t)}

running['v']=True
def bye(sig,f):
    running['v']=False; stop(); log("SIGTERM")
signal.signal(signal.SIGTERM,bye); signal.signal(signal.SIGINT,bye)

open('/memory/cmd.txt','w').close()
threading.Thread(target=poller,daemon=True).start()
threading.Thread(target=radio_thread,daemon=True).start()
log("pilot7 started pid "+str(os.getpid()))
last_cmd=time.time()
while running['v']:
    try:
        lines=open('/memory/cmd.txt').read().strip().split('\n')
        if lines and lines[0]:
            open('/memory/cmd.txt','w').close()
            last_cmd=time.time()
            for L in lines:
                try:
                    op=json.loads(L); res=handle(op,time.time())
                    log("CMD "+L[:100]+" -> "+json.dumps(res)[:500])
                except Exception as e:
                    log("CMD ERR "+L[:80]+": "+repr(e)); stop()
        else:
            time.sleep(0.1)
        if time.time()-last_cmd>12:
            with lock: g=state['d6']; r=state['d9']
            # safety: motors should be 0 between commands
    except Exception as e:
        log("loop err "+repr(e)); time.sleep(0.2)
stop(); log("pilot7 exited")
