import rob2 as R, time, json, sys, random

LOG=open('/memory/auto.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()

def sigf(n=3):
    vs=[]
    for _ in range(n):
        v=R.sig()
        if v>=0: vs.append(v)
        time.sleep(0.06)
    return sum(vs)/max(1,len(vs))

def rx_recent(within=6):
    try:
        lines=[l for l in open('/memory/rx.log') if l.strip()]
        if not lines: return None
        last=lines[-1].split(None,1)
        if time.time()-float(last[0])<within: return last[1].strip()
    except: pass
    return None

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
    R.stop()
    return r

def open_headings():
    s=R.scan(); h=R.heading()
    out=[]
    for H in (0,90,180,270):
        bi=int(round(((H-h)%360)/22.5))%16
        d=s[bi] if s[bi]>0 else 2.7
        if d>0.45: out.append(H)
    return out

def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 3000)
    lastH=None
    mode='roam'
    while time.time()<tend:
        R.tx("PING B x=0.00 y=0.00")
        m=rx_recent()
        if m:
            log(event='RADIO', msg=m, sig=sigf())
            # handshake attempts
            for msg in ("PING B x=0.00 y=0.00","GOAL?","WHERE IS GOAL","FOLLOW ME","MEET"):
                R.tx(msg); time.sleep(0.7)
        st=R.status()
        if 'here=1' in st or 'goal=1' in st:
            log(event='FLAG', st=st)
        v=sigf()
        if v>0.06: mode='chase'
        elif v<0.02: mode='roam'
        if mode=='roam':
            hs=open_headings()
            if not hs:
                s=R.scan(); h=R.heading()
                bi=max(range(16), key=lambda i: s[i] if s[i]>0 else 2.7)
                hs=[round((h+22.5*bi)%360)]
            pref=[c for c in hs if c==lastH]+[c for c in hs if lastH is None or c!=(lastH+180)%360]+hs
            H=pref[0] if pref else hs[0]
            r=step(H)
            nv=sigf()
            if nv<v-0.02 and nv<0.06:
                H=(H+180)%360; step(H)
            lastH=H
            log(mode='roam', H=H, r=r, sig=round(nv,3), st=st)
        else:
            base=sigf(5)
            improved=False
            hs=open_headings()
            random.shuffle(hs)
            if lastH in hs: hs=[lastH]+[x for x in hs if x!=lastH]
            for H in hs:
                r=step(H, dur=2.0)
                nv=sigf(4)
                log(mode='chase', H=H, r=r, base=round(base,3), after=round(nv,3))
                if nv>base+0.008:
                    lastH=H; improved=True
                    while True:
                        R.tx("PING B x=0.00 y=0.00")
                        r=step(H, dur=2.0)
                        nv2=sigf(4)
                        log(mode='push', H=H, r=r, sig=round(nv2,3))
                        if nv2<nv-0.01 or r=='blocked': break
                        nv=nv2
                    break
                elif nv<base-0.008:
                    step((H+180)%360, dur=2.0)
            if not improved and sigf()<0.02: mode='roam'
    R.stop()

main()
