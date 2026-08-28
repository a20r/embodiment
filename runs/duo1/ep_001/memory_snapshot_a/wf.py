import rob, time, sys

def bng(): return float(rob.rd(10))

def wall_follow(side=4, tmax=600, spd=18):
    t0=time.time(); o0=rob.odo()
    DW=0.22
    while time.time()-t0 < tmax:
        if rob.goal(): print("GOAL!", flush=True); rob.motors(0,0); return True
        L=rob.lidar()
        F=L[0]; D=L[side]
        diag = L[2] if side==4 else L[14]
        if D<0: D=DW
        if 0<F<0.28:
            rob.motors(0,0)
            if side==4: rob.motors(-20,20)
            else: rob.motors(20,-20)
            tt=time.time()
            while time.time()-tt<10:
                Lf=rob.lidar()
                if Lf[0]>0.45 or Lf[0]<0: break
                time.sleep(0.08)
            rob.motors(0,0)
            print(f"corner t={time.time()-t0:.0f} b={bng():.0f} odo={rob.odo()-o0}", flush=True)
            continue
        if D>0.5:
            if side==4: rob.motors(spd, spd*0.25)
            else: rob.motors(spd*0.25, spd)
            print(f"arc-in t={time.time()-t0:.0f} b={bng():.0f} D={D:.2f} odo={rob.odo()-o0}", flush=True)
            time.sleep(0.25)
            continue
        c1 = (D-DW)*50
        c2 = (diag-DW*1.15)*20 if diag>0 else 0
        corr = max(-8, min(8, c1 + c2))
        if side==4: rob.motors(spd+corr, spd-corr)
        else: rob.motors(spd-corr, spd+corr)
        time.sleep(0.1)
    rob.motors(0,0)
    print("WF timeout odo", rob.odo()-o0, flush=True)
    return False

if __name__=="__main__":
    side=int(sys.argv[1]) if len(sys.argv)>1 else 4
    print("start bearing", bng(), flush=True)
    wall_follow(side)
