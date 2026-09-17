import rob2 as R, time, json, sys

LOG=open('/memory/pounce.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()

def sigf(n=3):
    vs=[]
    for _ in range(n):
        v=R.sig()
        if v>=0: vs.append(v)
        time.sleep(0.07)
    return sum(vs)/max(1,len(vs))

def probe(H, dur=1.6):
    """move dur seconds toward H (if open), return sig after; None if blocked immediately."""
    h=R.heading()
    bi=int(round(((H-h)%360)/22.5))%16
    s=R.scan()
    d=s[bi] if s[bi]>0 else 2.7
    if d<0.4: return None
    R.turn_to(H, tol=6)
    t0=time.time()
    while time.time()-t0<dur:
        sc=R.scan()
        f=sc[0] if sc[0]>0 else 2.7
        if f<0.3: break
        hh=R.heading(); err=R.norm(H-hh)
        R.wr('d1',round(16+max(min(1.2*err,8),-8),1)); R.wr('d7',20)
        time.sleep(0.1)
    R.stop()
    return sigf()

def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 900)
    mode='wait'
    while time.time()<tend:
        R.tx("PING B x=0.00 y=0.00")
        v=sigf()
        st=R.status()
        if 'here=1' in st or 'goal=1' in st: log(event='FLAG',st=st)
        if v<0.18:
            log(mode='wait', sig=round(v,3), st=st)
            time.sleep(1.5)
            continue
        # pounce: greedy gradient with short probes
        log(mode='pounce-start', sig=round(v,3))
        fails=0
        while time.time()<tend:
            R.tx("PING B x=0.00 y=0.00")
            base=sigf(5)
            if base<0.10: break
            best=(None,base)
            for H in (0,90,180,270):
                r=probe(H)
                if r is None: continue
                log(probe=H, sig=round(r,3), base=round(base,3))
                if r>best[1]+0.01:
                    best=(H,r); break   # commit fast: keep moving this way
                else:
                    # go back
                    probe((H+180)%360)
            if best[0] is None:
                fails+=1
                if fails>2: break
            else:
                fails=0
                # keep pushing same direction while improving
                H=best[0]; cur=best[1]
                while True:
                    r=probe(H)
                    R.tx("PING B x=0.00 y=0.00")
                    if r is None or r<cur-0.01: break
                    cur=r
                    log(push=H, sig=round(r,3))
    R.stop()

main()
