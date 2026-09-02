import rob2 as R, time, json, sys, random

LOG=open('/memory/hunt.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()

def sigf(n=2):
    vs=[]
    for _ in range(n):
        v=R.sig()
        if v>=0: vs.append(v)
        time.sleep(0.05)
    return sum(vs)/max(1,len(vs))

def median_scan(k=4):
    bs=[R.scan() for _ in range(k)]
    out=[]
    for i in range(16):
        vals=sorted(b[i] for b in bs if b[i]>0)
        out.append(vals[len(vals)//2] if vals else 2.7)
    return out

def step(H, dur=4.5, stopd=0.32):
    R.turn_to(H, tol=6)
    t0=time.time(); r='time'
    while time.time()-t0<dur:
        sc=R.scan()
        f=sc[0] if sc[0]>0 else 2.7
        if f<stopd: r='blocked'; break
        hh=R.heading(); err=R.norm(H-hh)
        R.wr('d1',round(16+max(min(1.2*err,8),-8),1)); R.wr('d7',20)
        time.sleep(0.1)
    R.stop(); return r

def open_headings():
    s=R.scan(); h=R.heading()
    out=[]
    for H in (0,90,180,270):
        bi=int(round(((H-h)%360)/22.5))%16
        d=s[bi] if s[bi]>0 else 2.7
        if d>0.45: out.append(H)
    return out

def watch_and_dash(timeout=90):
    """Park; detect A by lidar change or sig rise; dash toward it. Return best sig seen."""
    R.stop(); time.sleep(0.4)
    base=median_scan(); h0=R.heading()
    t0=time.time(); best=0
    while time.time()-t0<timeout:
        R.tx("PING B x=0.00 y=0.00")
        st=R.status()
        if 'here=1' in st or 'goal=1' in st: log(event='FLAG', st=st)
        s=R.scan(); h=R.heading(); v=sigf()
        best=max(best,v)
        shift=int(round(R.norm(h-h0)/22.5))%16
        cand=[]
        for i in range(16):
            bi=(i+shift)%16
            if s[i]>0 and base[bi]-s[i]>0.35 and s[i]<1.6:
                cand.append((s[i],i))
        if cand and v>0.10:
            d,i=min(cand)
            tgt=(h+22.5*i)%360
            log(event='TARGET', dist=d, sig=round(v,3), tgt=round(tgt))
            R.turn_to(tgt, tol=8)
            te=time.time()+4
            while time.time()<te:
                R.tx("PING B x=0.00 y=0.00")
                sc=R.scan(); f=sc[0] if sc[0]>0 else 2.7
                if f<0.25: break
                hh=R.heading(); err=R.norm(tgt-hh)
                R.wr('d1',round(16+max(min(1.2*err,8),-8),1)); R.wr('d7',20)
                time.sleep(0.1)
            R.stop()
            nv=sigf(6); best=max(best,nv)
            log(event='dash-done', sig=round(nv,3), st=R.status())
            time.sleep(0.4); base=median_scan(); h0=R.heading(); t0=time.time()
        else:
            time.sleep(0.12)
    return best

def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 3000)
    lastH=None
    while time.time()<tend:
        v=sigf()
        R.tx("PING B x=0.00 y=0.00")
        if v>0.08:
            b=watch_and_dash(60)
            log(event='watch-end', best=round(b,3))
            continue
        # roam a few steps
        hs=open_headings()
        if not hs:
            s=R.scan(); h=R.heading()
            bi=max(range(16), key=lambda i: s[i] if s[i]>0 else 2.7)
            hs=[round((h+22.5*bi)%360)]
        pref=[c for c in hs if c==lastH]+[c for c in hs if lastH is None or c!=(lastH+180)%360]+hs
        H=pref[0]
        r=step(H)
        lastH=H
        nv=sigf()
        log(mode='roam', H=H, r=r, sig=round(nv,3))
        # every ~6 roam steps, park & listen 30s
        if random.random()<0.18:
            b=watch_and_dash(30)
            log(event='listen', best=round(b,3))
    R.stop()

main()
