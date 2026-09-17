import rob2 as R, time, json, sys, random

LOG=open('/memory/chase.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()

def sig():
    v=R.sig()
    return v if v>=0 else None

def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 600)
    lastH=None
    ema=None
    while time.time()<tend:
        st=R.status()
        if 'here=1' in st or 'goal=1' in st:
            log(event='FLAG', st=st); R.stop(); time.sleep(2); continue
        s=R.scan(); h=R.heading()
        # candidate open headings
        cands=[]
        for H in (0,90,180,270):
            bi=int(round(((H-h)%360)/22.5))%16
            d=s[bi] if s[bi]>0 else 2.7
            if d>0.45: cands.append(H)
        if not cands:
            bi=max(range(16), key=lambda i: s[i] if s[i]>0 else 2.7)
            cands=[round((h+22.5*bi)%360)]
        pref=[c for c in cands if c==lastH] + [c for c in cands if c!=lastH and (lastH is None or c!=(lastH+180)%360)] + [c for c in cands]
        H=pref[0]
        R.turn_to(H)
        # drive while monitoring sig trend
        t0=time.time(); samples=[]
        e_stop='time'
        R.tx("PING B")
        while time.time()-t0<6:
            sc=R.scan()
            f=sc[0] if sc[0]>0 else 2.7
            if f<0.32: e_stop='blocked'; break
            hh=R.heading(); err=R.norm(H-hh)
            R.wr('d1',round(16+max(min(1.2*err,8),-8),1)); R.wr('d7',20)
            v=sig()
            if v is not None: samples.append(v)
            if len(samples)>14:
                a=sum(samples[-7:])/7; b=sum(samples[-14:-7])/7
                if a<b-0.02: e_stop='sigdrop'; break
        R.stop()
        first=sum(samples[:5])/max(1,len(samples[:5])) if samples else -1
        last=sum(samples[-5:])/max(1,len(samples[-5:])) if samples else -1
        log(H=H, stop=e_stop, s0=round(first,3), s1=round(last,3), st=R.status())
        if e_stop=='sigdrop':
            lastH=(H+180)%360  # turn around next
        else:
            lastH=H
    R.stop()

main()
