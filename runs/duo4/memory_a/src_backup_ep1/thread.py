import time, math, sys
sys.path.insert(0,'/bot/src')
from bot import IO, clean
io=IO()
log=open('/memory/thread.log','a',buffering=1)
T0=time.time()
prev=[None]; lastping=[0.0]
def now(): return time.time()-T0
def wrap(a): return (a+180)%360-180
d6hist=[]
def d6avg():
    return sum(d6hist[-8:])/max(1,len(d6hist[-8:]))
def poll(t=0.03):
    io.poll(t)
    v=io.latest.get(6)
    if v is not None:
        try:
            d6hist.append(float(v))
            if len(d6hist)>400: del d6hist[:200]
        except: pass
    tn=time.time()
    if tn-lastping[0]>2:
        lastping[0]=tn
        io.send('ALPHA: following your beacon. goalflag=%s'%('1' if 'goal=1' in io.latest.get(9,'') else '0'))
        st=io.latest.get(9,'')
        log.write('%.1f HB d6=%s %s\n'%(now(),io.latest.get(6),st))
        if 'goal=1' in st: log.write('%.1f GOAL!\n'%now())
    for m in io.msgs:
        log.write('%.1f RX %s\n'%(now(),m)); print('RX',m,flush=True)
    io.msgs=[]
def sensors(dur=0.04):
    t=time.time()
    while time.time()-t<dur: poll()
    l=io.lidar(); h=io.heading()
    while l is None or h is None:
        poll(); l=io.lidar(); h=io.heading()
    l=clean(l,prev[0]); prev[0]=l
    return l,h
def turn_to(tgt,tol=4):
    t0=time.time()
    while time.time()-t0<9:
        l,h=sensors(0.02)
        d=wrap(tgt-h)
        if abs(d)<tol: io.drive(0,0); return True
        mag=min(90,max(14,abs(d)*1.6))
        io.drive(-mag if d>0 else mag,0)
        time.sleep(0.02)
    io.drive(0,0); return False

def scan():
    """rotate 360, return 72-bin world polar map"""
    bins=[[] for _ in range(72)]
    io.drive(-40,0)
    t0=time.time(); lasth=None; acc=0
    while acc<370 and time.time()-t0<15:
        l,h=sensors(0.02)
        if lasth is not None: acc+=abs(wrap(h-lasth))
        lasth=h
        for i,v in enumerate(l):
            if v<0: continue
            wd=(h+i*22.5)%360
            bins[int(wd//5)%72].append(v)
    io.drive(0,0)
    out=[]
    for b in bins:
        out.append(min(b) if b else None)
    # fill gaps
    for i in range(72):
        if out[i] is None:
            out[i]=out[(i-1)%72] if out[(i-1)%72] is not None else 0.3
    return out

def gaps(pm):
    """find angular windows passable: need clearance 0.30m"""
    res=[]
    n=72
    i=0
    vis=[d>0.36 for d in pm]
    used=[False]*n
    for s in range(n):
        if vis[s] and not used[s]:
            e=s
            while vis[(e+1)%n] and (e+1-s)<n: e+=1; used[e%n]=True
            used[s]=True
            width=(e-s+1)*5
            seg=[pm[k%n] for k in range(s,e+1)]
            dmin=min(seg)
            span=math.radians(width)*dmin
            centre=((s+e)/2*5+2.5)%360
            res.append(dict(dir=centre,width=width,dmin=dmin,dmax=max(seg),span=span))
    return sorted(res,key=lambda g:-g['dmax'])

def thread(wd, maxt=12):
    """squeeze along world dir wd"""
    turn_to(wd,3)
    t0=time.time(); lastsnap=None; lastsnapt=time.time(); moved=False
    spd=10
    d0=d6avg(); lastchk=time.time()
    while time.time()-t0<maxt:
        l,h=sensors(0.02)
        f=min(l[0],l[1],l[15])
        if f<0.24:
            io.drive(0,-12); time.sleep(0.12); io.drive(0,0)
            return 'wall' if moved else 'wall_nomove'
        dh=wrap(wd-h)
        turn=-2.2*dh
        lft=min(l[3],l[4]); rgt=min(l[12],l[13])
        if lft<0.45 and rgt<0.45: turn+=80*(rgt-lft)
        elif lft<0.16: turn+=25
        elif rgt<0.16: turn-=25
        io.drive(max(-45,min(45,turn)),spd)
        t=time.time()
        if t-lastchk>2.5:
            lastchk=t; cur=d6avg()
            if cur<d0-0.05 and cur<0.75:
                io.drive(0,0); return 'worse'
            if cur>d0+0.03:
                d0=cur; t0=min(t0+6,t)  # extend time while improving
        if lastsnap is None or t-lastsnapt>1.0:
            if lastsnap is not None:
                d=max(abs(a-b) for a,b in zip(l,lastsnap))
                if d<0.05:
                    if spd<44: spd+=11   # escalate to break friction
                    else:
                        io.drive(0,0); return 'stuck' if not moved else 'stuck_after'
                else:
                    moved=True
                    import math as _m
                    step=0.036*spd*1.0
                    X[0]+=step*_m.cos(_m.radians(h)); Y[0]+=step*_m.sin(_m.radians(h))
                    mark()
                    samples.append((X[0],Y[0],d6avg()))
                    if len(samples)>60: del samples[:30]
                    spd=10
            lastsnap=l; lastsnapt=t
        time.sleep(0.02)
    io.drive(0,0); return 'time'

import json
X=[0.0]; Y=[0.0]; visits={}
samples=[]
def cell(x,y): return (int(round(x*2)),int(round(y*2)))
def mark():
    c=cell(X[0],Y[0]); visits[c]=visits.get(c,0)+1
tried={}
lastdir=None
gooddir=None
atgoal=[False]
while True:
    st=io.latest.get(9,'')
    if 'goal=1' in st:
        io.drive(0,0)
        if not atgoal[0]:
            atgoal[0]=True; log.write('%.1f AT GOAL, parking\n'%now())
        io.send('ALPHA AT GOAL - COME TO MY BEACON')
        poll(0.3)
        continue
    else:
        if atgoal[0]:
            log.write('%.1f left goal?? resuming\n'%now()); atgoal[0]=False
    # hold-off: if very close to BETA, wait so we don't block it
    if False:
        io.drive(0,0)
        t0=time.time()
        while d6avg()>0.80 and time.time()-t0<25:
            poll(0.1)
        log.write('%.1f WAIT done d6=%.3f\n'%(now(),d6avg()))
    pm=scan()
    gs=gaps(pm)
    log.write('%.1f SCAN %s\n'%(now(),json.dumps([{k:(round(v,2) if isinstance(v,float) else v) for k,v in g.items()} for g in gs[:6]])))
    cand=None
    best=-1
    rh = d6avg()>0.78 and lastdir is not None
    for g in gs:
        key=int(g['dir']//30)
        if g['dmax']>0.45 and tried.get(key,0)<2:
            sc=g['dmax']
            import math as _m
            if rh:
                pref=(lastdir-90)%360
                a=abs(wrap(g['dir']-pref))
                sc=3.0-a/90.0+0.3*g['dmax']
                if sc>best: best=sc; cand=g
                continue
            px=X[0]+1.0*_m.cos(_m.radians(g['dir'])); py=Y[0]+1.0*_m.sin(_m.radians(g['dir']))
            sc/= (1+0.12*visits.get(cell(px,py),0))
            if gooddir is not None:
                sc*=(2.4-abs(wrap(g['dir']-gooddir))/75)
            elif lastdir is not None:
                sc*=(1.6-abs(wrap(g['dir']-lastdir))/180)
            if sc>best: best=sc; cand=g
    if cand is None:
        tried={}
        cand=gs[0] if gs else None
        if cand is None:
            io.drive(0,-20); time.sleep(0.6); io.drive(0,0); continue
    import os as _os
    exc=_os.environ.get('EXCURSION')
    if exc: gooddir=float(exc)
    # regression gradient estimate
    if (not exc) and len(samples)>=8:
        import math as _m
        pts=samples[-20:]
        mx=sum(p[0] for p in pts)/len(pts); my=sum(p[1] for p in pts)/len(pts); mv=sum(p[2] for p in pts)/len(pts)
        sxx=sum((p[0]-mx)**2 for p in pts)+1e-6; syy=sum((p[1]-my)**2 for p in pts)+1e-6
        sxv=sum((p[0]-mx)*(p[2]-mv) for p in pts); syv=sum((p[1]-my)*(p[2]-mv) for p in pts)
        gx=sxv/sxx; gy=syv/syy
        if abs(gx)+abs(gy)>0.005:
            gooddir=_m.degrees(_m.atan2(gy,gx))%360
            log.write('%.1f GRAD %.0f\n'%(now(),gooddir))
    d6a=d6avg()
    res=thread(cand['dir'],maxt=12)
    d6b=d6avg()
    log.write('%.1f THREAD dir=%.0f res=%s d6 %.3f->%.3f xy=%.1f,%.1f\n'%(now(),cand['dir'],res,d6a,d6b,X[0],Y[0]))
    if exc: gooddir=float(exc)
    elif d6b>d6a+0.015: gooddir=cand['dir']
    elif d6b<d6a-0.015: gooddir=(cand['dir']+180)%360
    if res in ('stuck','wall_nomove'):
        tried[int(cand['dir']//30)]=tried.get(int(cand['dir']//30),0)+1
        lastdir=None
    else:
        tried={}
        lastdir=cand['dir']
