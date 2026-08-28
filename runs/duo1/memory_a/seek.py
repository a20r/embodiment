import rob, time, random

def bng(): return float(rob.rd(10))
def serr(b, ref=0): return (b - ref + 540) % 360 - 180

def turn_to_reading(ref, tol=6, timeout=20):
    t0=time.time()
    while time.time()-t0<timeout:
        b=bng(); e=serr(b,ref)
        if abs(e)<tol: rob.motors(0,0); return True
        s=max(6,min(50,abs(e)))
        if e>0: rob.motors(s,-s)
        else: rob.motors(-s,s)
        time.sleep(0.1)
    rob.motors(0,0); return False

def dive():
    """drive toward goal until blocked. returns when front blocked."""
    while True:
        if rob.goal(): return "GOAL"
        turn_to_reading(0)
        L=rob.lidar()
        if 0<L[0]<0.3: return "blocked"
        rob.motors(16,16)
        while True:
            L=rob.lidar()
            if (0<L[0]<0.3) or (0<L[1]<0.16) or (0<L[15]<0.16):
                rob.motors(0,0); break
            b=bng()
            if abs(serr(b,0))>25: rob.motors(0,0); break
            time.sleep(0.05)
        if rob.goal(): return "GOAL"
        L=rob.lidar()
        if 0<L[0]<0.3: return "blocked"

def follow(side, tmax, spd=18):
    """wall follow; break when goal-side beam opens. side=4 wall&goal right, 12 left."""
    ref = 90 if side==4 else 270
    t0=time.time(); opens=0
    DW=0.22
    while time.time()-t0<tmax:
        if rob.goal(): return "GOAL"
        L=rob.lidar(); b=bng()
        gb=int(round((b%360)/22.5))%16
        gd=L[gb]
        if gd>0.5 and abs(serr(b,ref))<70:
            opens+=1
            if opens>=2:
                rob.motors(0,0); return "open"
        else: opens=0
        F=L[0]; D=L[side]
        diag=L[2] if side==4 else L[14]
        if D<0: D=DW
        if 0<F<0.28:
            if side==4: rob.motors(-20,20)
            else: rob.motors(20,-20)
            tt=time.time()
            while time.time()-tt<10:
                Lf=rob.lidar()
                if Lf[0]>0.45 or Lf[0]<0: break
                time.sleep(0.08)
            rob.motors(0,0)
            continue
        if D>0.5:
            if side==4: rob.motors(spd,spd*0.25)
            else: rob.motors(spd*0.25,spd)
            time.sleep(0.22)
            continue
        c=(D-DW)*50 + ((diag-DW*1.15)*20 if diag>0 else 0)
        c=max(-8,min(8,c))
        if side==4: rob.motors(spd+c,spd-c)
        else: rob.motors(spd-c,spd+c)
        time.sleep(0.1)
    rob.motors(0,0); return "tmax"

def main():
    t0=time.time()
    side=4
    while time.time()-t0<1800:
        r=dive()
        print(f"dive->{r} b={bng():.0f} t={time.time()-t0:.0f}", flush=True)
        if r=="GOAL": print("GOAL!!!", flush=True); return
        # follow wall keeping goal on chosen side; random duration to break cycles
        dur=random.uniform(15,50)
        r=follow(side, dur)
        print(f"follow(side={side},{dur:.0f}s)->{r} b={bng():.0f}", flush=True)
        if r=="GOAL": print("GOAL!!!", flush=True); return
        if r=="tmax" and random.random()<0.35:
            side = 16-side  # switch 4<->12
            print("switch side ->", side, flush=True)
    print("giving up", flush=True)

if __name__=="__main__":
    main()
