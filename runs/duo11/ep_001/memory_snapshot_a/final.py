import rob2 as R, time, json, sys
LOG=open('/memory/final.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()
CLAIM="PING B x=23.80 y=15.90"
def sigf(n=6):
    vs=[R.sig() for _ in range(n)]; vs=[v for v in vs if v>=0]; vs.sort()
    m=vs[1:-1] if len(vs)>3 else vs
    return sum(m)/max(1,len(m))
def step(H, dur=2.2, stopd=0.3):
    R.turn_to(H, tol=6)
    t0=time.time(); r='time'
    while time.time()-t0<dur:
        sc=R.scan(); f=sc[0] if sc[0]>0 else 2.7
        if f<stopd: r='blocked'; break
        hh=R.heading(); err=R.norm(H-hh)
        R.wr('d1',round(16+max(min(1.2*err,8),-8),1)); R.wr('d7',20)
        time.sleep(0.1)
    R.stop(); return r
def seek(tlimit=240, target=0.5):
    t0=time.time(); H=0
    while time.time()-t0<tlimit:
        R.tx(CLAIM)
        st=R.status()
        if 'here=1' in st or 'goal=1' in st: log(event='FLAG', st=st)
        v=sigf()
        if v>=target: return v
        s=R.scan(); h=R.heading()
        moved=False
        if v>0.08:
            for HH in (0,90,180,270):
                bi=int(round(((HH-h)%360)/22.5))%16
                if (s[bi] if s[bi]>0 else 2.7)<0.45: continue
                r=step(HH)
                nv=sigf()
                log(seek=HH, r=r, v=round(v,3), nv=round(nv,3))
                if nv>v-0.005:
                    moved=True; H=HH; break
                step((HH+180)%360)
                s=R.scan(); h=R.heading()
        if not moved:
            for cand in [(H+90)%360, H, (H-90)%360, (H+180)%360]:
                bi=int(round(((cand-h)%360)/22.5))%16
                if (s[bi] if s[bi]>0 else 2.7)>0.5:
                    H=cand; break
            r=step(H)
            log(wf=H, r=r, sig=round(sigf(),3))
    return sigf()
def park(tlimit=600):
    t0=time.time(); lowsince=None
    while time.time()-t0<tlimit:
        R.tx(CLAIM)
        v=sigf(3); st=R.status()
        if 'here=1' in st or 'goal=1' in st: log(event='FLAG', st=st)
        if v>0.6: log(park_sig=round(v,3), st=st)
        if v<0.1:
            lowsince=lowsince or time.time()
            if time.time()-lowsince>150: return 'lost'
        else: lowsince=None
        time.sleep(0.9)
    return 'timeout'
def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 2400)
    while time.time()<tend:
        v=seek(240, 0.5)
        log(event='seek-end', sig=round(v,3))
        r=park(500)
        log(event='park-end', r=r)
    R.stop()
main()
