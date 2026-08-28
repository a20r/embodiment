import rob, ctl, time, sys

def bng():
    return float(rob.rd(10))

def serr(b, ref):  # signed error of bearing vs ref
    return (b - ref + 540) % 360 - 180

def turn_reading_to(ref, tol=6, timeout=25):
    t0=time.time()
    while time.time()-t0<timeout:
        e = serr(ctl.h_avg(3), ref)
        if abs(e) < tol:
            rob.motors(0,0); return True
        s = max(6, min(50, abs(e)))
        # motors(l>r) decreases reading
        if e > 0: rob.motors(s,-s)
        else:     rob.motors(-s,s)
        time.sleep(0.12)
    rob.motors(0,0); return False

def face_goal(): return turn_reading_to(0)

def goal_beam(b):
    return int(round((b % 360)/22.5)) % 16

def dive():
    """goal-ward drive as far as possible"""
    print("DIVE", flush=True)
    while True:
        if rob.goal(): print("GOAL!", flush=True); return True
        face_goal()
        L = rob.lidar()
        if 0 < L[0] < 0.3:
            rob.motors(0,0); print(f"dive blocked F={L[0]:.2f}", flush=True); return False
        rob.motors(12,12)
        o0=rob.odo()
        while rob.odo()-o0 < 40:
            L=rob.lidar()
            if (0<L[0]<0.28) or (0<L[1]<0.15) or (0<L[15]<0.15):
                break
            time.sleep(0.04)
        rob.motors(0,0)
        if rob.goal(): print("GOAL!", flush=True); return True

def orbit(side=90, tmax=110):
    """drive along ring keeping goal at bearing `side` (90: goal right-ish)."""
    t0=time.time()
    turn_reading_to(side)
    o_start=rob.odo()
    while time.time()-t0 < tmax:
        if rob.goal(): print("GOAL!", flush=True); return True
        b = bng(); e = serr(b, side)
        L = rob.lidar()
        gb = goal_beam(b)
        gd = L[gb]
        # door toward goal?
        if gd > 0.55 or gd < 0:
            rob.motors(0,0)
            print(f"door? bearing={b:.0f} beam{gb}={gd:.2f} odo={rob.odo()-o_start}", flush=True)
            if dive(): return True
            turn_reading_to(side)
            continue
        if 0 < L[0] < 0.3:
            # corner: realign
            rob.motors(0,0)
            turn_reading_to(side, tol=8)
            L=rob.lidar()
            if 0 < L[0] < 0.3:
                # still blocked: nudge reading outward (goal more behind)
                print(f"corner stuck F={L[0]:.2f} b={b:.0f}", flush=True)
                turn_reading_to((side+30)%360, tol=8)
            continue
        corr = max(-10, min(10, -e*0.6))
        # to decrease reading (e>0) need l>r => corr negative adds to left
        rob.motors(14-corr, 14+corr)
        time.sleep(0.12)
    rob.motors(0,0)
    print("orbit timeout, odo", rob.odo()-o_start, flush=True)
    return False

if __name__=="__main__":
    side = int(sys.argv[1]) if len(sys.argv)>1 else 90
    if orbit(side): print("DONE", flush=True)
