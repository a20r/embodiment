import rob2 as R, time, json, sys

LOG=open('/memory/intercept.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()

def sigf(n=2):
    vs=[]
    for _ in range(n):
        v=R.sig()
        if v>=0: vs.append(v)
        time.sleep(0.05)
    return sum(vs)/max(1,len(vs))

def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 1200)
    R.stop()
    time.sleep(0.5)
    base=[R.scan() for _ in range(5)]
    base=[sorted(b[i] for b in base if b[i]>0)[len([1 for b in base if b[i]>0])//2] if any(b[i]>0 for b in base) else 2.7 for i in range(16)]
    log(event='baseline', scan=[round(x,2) for x in base])
    h0=R.heading()
    while time.time()<tend:
        R.tx("PING B x=0.00 y=0.00")
        st=R.status()
        if 'here=1' in st or 'goal=1' in st: log(event='FLAG', st=st)
        s=R.scan(); h=R.heading()
        v=sigf()
        # rotate baseline by heading change
        shift=int(round(R.norm(h-h0)/22.5))%16
        moved=[]
        for i in range(16):
            bi=(i+shift)%16   # approx
            if s[i]>0 and base[bi]-s[i]>0.35 and s[i]<1.5:
                moved.append((i,s[i]))
        if moved and v>0.12:
            i,d=min(moved,key=lambda t:t[1])
            tgt=(h+22.5*i)%360
            log(event='TARGET', beam=i, dist=d, sig=round(v,3), tgt=round(tgt))
            # dash toward it
            R.turn_to(tgt, tol=8)
            t0=time.time()
            while time.time()-t0<4:
                R.tx("PING B x=0.00 y=0.00")
                sc=R.scan()
                f=sc[0] if sc[0]>0 else 2.7
                if f<0.25: break
                hh=R.heading(); err=R.norm(tgt-hh)
                R.wr('d1',round(16+max(min(1.2*err,8),-8),1)); R.wr('d7',20)
                time.sleep(0.1)
            R.stop()
            nv=sigf(6)
            log(event='dash-done', sig=round(nv,3), st=R.status())
            # re-baseline here
            time.sleep(0.5)
            bs=[R.scan() for _ in range(4)]
            base=[sorted(b[i] for b in bs if b[i]>0)[len([1 for b in bs if b[i]>0])//2] if any(b[i]>0 for b in bs) else 2.7 for i in range(16)]
            h0=R.heading()
        else:
            if int(time.time())%10==0: log(sig=round(v,3))
            time.sleep(0.15)
    R.stop()

main()
