import os, sys, time, json, select, re, signal

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

def stop():
    write_motors(0,0)

def norm180(a,b): return (b-a+540)%360-180

def grab_scan(dur=1.2):
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
    return pts

def scan_summary(pts):
    out={}
    for tag,g in (('pos',[p for p in pts if p[0]>=0]),('neg',[p for p in pts if p[0]<0])):
        if not g: out[tag]=None; continue
        bins={}
        for r,c1,c2 in g:
            b=round(c2/0.05)*0.05
            v=abs(r)
            if b not in bins or v<bins[b][0]: bins[b]=(v,c1)
        out[tag]={str(k):[round(v,3),round(c,3)] for k,(v,c) in sorted(bins.items())}
    return out

def do_scan():
    pts=grab_scan(1.2)
    s=scan_summary(pts)
    json.dump(s,open('/memory/scan_latest.json','w'))
    pos=[p for p in pts if p[0]>=0]; neg=[p for p in pts if p[0]<0]
    fp = min((abs(p[0]) for p in pos if abs(p[2])<0.1), default=None)
    return {"pts":len(pts),"pos":len(pos),"neg":len(neg),
            "pos_rmin":fp, "grid":s}

def do_rot(deg,cmd=50,dirn=1):
    # dirn=1: L=+cmd R=-cmd ... determine sign empirically: L50R-50 gave heading -174 (CCW)
    h0=float(last_of('d4',0.15))
    tgt=h0+deg
    s=1 if deg>0 else -1
    # L-50 R50 -> heading + (CW). So for deg>0: L=-cmd, R=+cmd
    lt, rt = (-cmd*s, cmd*s)
    t0=time.time()
    write_motors(lt,rt)
    err=999
    while time.time()-t0<abs(deg)/(1.74*cmd)*2+2:
        h=float(last_of('d4',0.1))
        if h is None: continue
        err=norm180(tgt,h)
        if abs(err)<3: break
    stop()
    h1=float(last_of('d4',0.15))
    return {"rot_done":round(norm180(h0,h1),1),"target":round(deg,1)}

def do_fwd(sec,cmd=40):
    e0l=last_of('d6',0.12); e0r=last_of('d9',0.12)
    t0=time.time(); write_motors(cmd,cmd)
    while time.time()-t0<sec:
        st=read1('d3',0.05)
        ln=lastline(st) if st else None
        if ln and 'goal=1' in ln:
            log("GOAL FLAG SEEN during fwd!"); break
    stop()
    e1l=last_of('d6',0.12); e1r=last_of('d9',0.12)
    try: dl=int(e1l)-int(e0l); dr=int(e1r)-int(e0r)
    except: dl=dr=-1
    return {"ticks_l":dl,"ticks_r":dr}

def do_tx(msg):
    fd=os.open(R+'d8',os.O_WRONLY|os.O_NONBLOCK)
    n=os.write(fd,(msg+"\n").encode()); os.close(fd)
    return {"tx":msg,"n":n}

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
    return {"err":"unknown op "+str(t)}

running=True
def bye(sig,f): 
    global running; running=False; stop(); log("SIGTERM -> stopped")
signal.signal(signal.SIGTERM,bye)
signal.signal(signal.SIGINT,bye)

open('/memory/cmd.txt','w').close()
log("pilot started, pid "+str(os.getpid()))
i=0
lastgoal=None; lasthere=None
while running:
    # sensor round robin
    p=READ_PORTS[i%len(READ_PORTS)]
    txt=read1(p,0.05)
    ln=lastline(txt) if txt else None
    if ln:
        state[p]=ln
        if p=='d3':
            try:
                d=dict(kv.split('=') for kv in ln.split() if '=' in kv)
                state['d3_goal']=int(d.get('goal',-1)); state['d3_here']=int(d.get('here',-1))
                state['tick']=int(d.get('tick',-1))
                if lastgoal is not None and state['d3_goal']!=lastgoal:
                    log(f"GOAL FLAG CHANGE: goal={state['d3_goal']} here={state['d3_here']} tick={state['tick']}")
                if lasthere is not None and state['d3_here']!=lasthere:
                    log(f"HERE FLAG CHANGE: goal={state['d3_goal']} here={state['d3_here']} tick={state['tick']}")
                lastgoal=state['d3_goal']; lasthere=state['d3_here']
            except Exception as e: pass
        if p=='d10' and ln.strip():
            log("RADIO RX: "+ln.strip())
            open('/memory/rx.log','a').write(ln.strip()+'\n')
    i+=1
    # commands
    try:
        lines=open('/memory/cmd.txt').read().strip().split('\n')
        if lines and lines[0]:
            open('/memory/cmd.txt','w').close()
            for L in lines:
                try:
                    op=json.loads(L)
                    res=handle(op)
                    log("CMD "+L+" -> "+json.dumps(res)[:400])
                except Exception as e:
                    log("CMD ERR "+L+" : "+repr(e))
                    stop()
    except Exception as e:
        log("cmdfile err "+repr(e))
    if i%60==0:
        json.dump(state,open('/memory/state.json','w'))
stop(); log("pilot exited")
