import rob2 as R, time, json, sys
LOG=open('/memory/wf.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()
def sigf(n=4):
    vs=[R.sig() for _ in range(n)]; vs=[v for v in vs if v>=0]
    return sum(vs)/max(1,len(vs))
def step(H, dur=2.4, stopd=0.3):
    R.turn_to(H, tol=6)
    t0=time.time(); r='time'
    while time.time()-t0<dur:
        sc=R.scan(); f=sc[0] if sc[0]>0 else 2.7
        if f<stopd: r='blocked'; break
        hh=R.heading(); err=R.norm(H-hh)
        R.wr('d1',round(16+max(min(1.2*err,8),-8),1)); R.wr('d7',20)
        time.sleep(0.1)
    R.stop(); return r
def openness(s,h,H):
    bi=int(round(((H-h)%360)/22.5))%16
    vals=[s[bi]]+[s[(bi+k)%16] for k in (-1,1)]
    vals=[v if v>0 else 2.7 for v in vals]
    return vals[0]
def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 3000)
    H=0
    n=0
    while time.time()<tend:
        n+=1
        R.tx("PING B x=0.00 y=0.00")
        v=sigf(); st=R.status()
        if 'here=1' in st or 'goal=1' in st: log(event='FLAG', st=st)
        if v>0.85:
            log(event='ADJACENT', sig=round(v,3), st=st)
            R.tx("PING B x=0.00 y=0.00"); time.sleep(0.8)
            continue
        s=R.scan(); h=R.heading()
        # right-hand rule: right of current H is H+90 (beam4 side)
        order=[(H+90)%360, H, (H-90)%360, (H+180)%360]
        for cand in order:
            if openness(s,h,cand)>0.5:
                H=cand; break
        r=step(H)
        v2=sigf()
        log(n=n, H=H, r=r, sig=round(v2,3), st=st)
    R.stop()
main()
