import rob2 as R, time, json, sys

LOG=open('/memory/fine.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()

def sig_avg(n=12):
    vals=[]
    for _ in range(n):
        v=R.sig()
        if v>=0: vals.append(v)
        time.sleep(0.08)
    vals.sort()
    return sum(vals[2:-2])/len(vals[2:-2]) if len(vals)>6 else sum(vals)/max(1,len(vals))

def main():
    tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 400)
    while time.time()<tend:
        base=sig_avg()
        st=R.status()
        log(base=round(base,3), st=st)
        R.tx(f"HELLO sig={base:.3f}")
        if 'here=1' in st or 'goal=1' in st:
            log(event='FLAG', st=st); break
        s=R.scan(); h=R.heading()
        best=(None,base)
        results={}
        for H in (0,90,180,270):
            bi=int(round(((H-h)%360)/22.5))%16
            d=s[bi] if s[bi]>0 else 2.7
            if d<0.45: continue
            R.turn_to(H)
            b0s,b0e,reason=R.fwd_step(H, maxtime=5)
            if reason=='stuck': R.unstick(H)
            v=sig_avg()
            results[H]=round(v,3)
            # return to center
            Hb=(H+180)%360
            R.turn_to(Hb)
            R.fwd_step(Hb, maxtime=5)
            if v>best[1]+0.01: best=(H,v)
            s=R.scan(); h=R.heading()
        log(results=results, best=best)
        if best[0] is not None:
            R.turn_to(best[0])
            R.fwd_step(best[0], maxtime=5)
        else:
            log(event='local max', base=round(base,3), st=R.status())
            # stay and broadcast
            for _ in range(10):
                R.tx("AT LOCAL MAX sig=%.3f"%base)
                time.sleep(1)
    R.stop()

main()
