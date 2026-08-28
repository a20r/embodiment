import rob, walker, time, sys

def drain(dur=0.0):
    out=''
    te=time.time()
    while True:
        s=rob.rd(5,tries=1)
        if s: out+=s; te=time.time()
        elif time.time()-te>dur: break
    return out

def wall_follow_hail(side=12, tmax=560, spd=20):
    t0=time.time(); DW=0.22; lasttx=0
    while time.time()-t0<tmax:
        if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); return
        now=time.time()
        if now-lasttx>0.5:
            rob.wr(6,"hello"); lasttx=now
        r=drain()
        if r:
            print(f"CONTACT {r!r} t={now-t0:.0f} b={walker.bng():.0f}", flush=True)
            rob.motors(0,0)
            # converse: keep hailing while stationary
            for msg in ("hello","hello","ping","who","help","follow","goal","open"):
                rob.wr(6,msg); time.sleep(0.6)
                rr=drain(1.0)
                print(f"  {msg} -> {rr!r}", flush=True)
        L=rob.lidar()
        F=L[0]; D=L[side]
        diag=L[14] if side==12 else L[2]
        if D<0: D=DW
        if 0<F<0.28:
            rob.motors(20,-20) if side==12 else rob.motors(-20,20)
            tt=time.time()
            while time.time()-tt<10:
                Lf=rob.lidar()
                if Lf[0]>0.45 or Lf[0]<0: break
                time.sleep(0.08)
            rob.motors(0,0); continue
        if D>0.5:
            if side==12: rob.motors(spd*0.25,spd)
            else: rob.motors(spd,spd*0.25)
            time.sleep(0.22); continue
        c=(D-DW)*50 + ((diag-DW*1.15)*20 if diag>0 else 0)
        c=max(-8,min(8,c))
        if side==12: rob.motors(spd-c,spd+c)
        else: rob.motors(spd+c,spd-c)
        time.sleep(0.09)
    rob.motors(0,0); print("end", flush=True)

if __name__=="__main__":
    wall_follow_hail(int(sys.argv[1]) if len(sys.argv)>1 else 12)
