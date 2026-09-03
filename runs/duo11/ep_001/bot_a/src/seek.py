import rob2 as R, time, math, json, sys, random

LOG=open('/memory/seek.log','a')
def log(**kw):
    kw['t']=round(time.time(),1)
    LOG.write(json.dumps(kw)+'\n'); LOG.flush()

def sig_avg(n=4):
    vals=[]
    for _ in range(n):
        v=R.sig()
        if v>=0: vals.append(v)
        time.sleep(0.1)
    return sum(vals)/max(1,len(vals))

def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 300)
    lastH=None
    sg=sig_avg()
    while time.time()<tend:
        s=R.scan(); h=R.heading(); st=R.status()
        R.tx(f"HELLO sig={sg:.3f}")
        if 'here=1' in st or 'goal=1' in st:
            log(event='FLAG', st=st); R.stop()
        def beam_for(H): return int(round(((H-h)%360)/22.5))%16
        cands=[]
        for H in (0,90,180,270):
            bi=beam_for(H)
            d=s[bi] if s[bi]>0 else 2.7
            if d>0.5: cands.append((H,d))
        if not cands:
            bi=max(range(16), key=lambda i: s[i] if s[i]>0 else 2.7)
            cands=[(round((h+22.5*bi)%360), s[bi])]
        random.shuffle(cands)
        cands.sort(key=lambda c: (0 if c[0]==lastH else (2 if lastH is not None and c[0]==(lastH+180)%360 else 1)))
        base=sg
        for H,d in cands:
            R.turn_to(H)
            b0s,b0e,reason=R.fwd_step(H, maxtime=6)
            time.sleep(0.2)
            new=sig_avg()
            moved=b0s-b0e if (b0s<2.65 and b0e<2.65) else 0.5
            log(H=H, moved=round(moved,2), reason=reason, sig_before=round(base,3), sig_after=round(new,3), st=R.status())
            if reason=='stuck': R.unstick(H)
            if new>base+0.005:
                sg=new; lastH=H; break
            elif new<base-0.005:
                Hb=(H+180)%360
                R.turn_to(Hb)
                R.fwd_step(Hb, maxtime=6)
                sg=sig_avg()
            else:
                sg=new; lastH=H; break
    R.stop()

main()
