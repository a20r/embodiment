import rob, ctl, time, sys

FRONT_STOP = 0.28
SIDE = 0.4

def bearing():
    return ctl.h_avg(3)  # d10 avg

def berr():
    b = bearing()
    return (b+180)%360-180   # signed: 0 = goal ahead

def face_goal():
    # rotate until d10 ~ 0
    for _ in range(40):
        e = berr()
        if abs(e) < 5:
            rob.motors(0,0); return True
        s = max(6, min(60, abs(e)*1.0))
        # to change d10 reading: turning with (l>r) decreases d10-as-heading reading;
        # d10 = bearing to goal; rotating robot CW(l>r?) earlier decreased reading.
        # earlier: motors(l>r) decreased d10. bearing err e>0 means goal at +e; we want reading->0 i.e. decrease reading if e>0
        if e > 0: rob.motors(s,-s)
        else:     rob.motors(-s,s)
        time.sleep(0.12)
    rob.motors(0,0); return False

def drive_step(maxodo=60, spd=15):
    o0 = rob.odo()
    rob.motors(spd,spd)
    while rob.odo()-o0 < maxodo:
        L = rob.lidar()
        if 0 < L[0] < FRONT_STOP or 0 < L[1] < 0.2 or 0 < L[15] < 0.2:
            rob.motors(0,0); return False, L
        time.sleep(0.05)
    rob.motors(0,0)
    return True, rob.lidar()

def main():
    t0=time.time()
    mode = "goto"
    wall_side = None
    while time.time()-t0 < 100:
        if rob.goal():
            print("GOAL!", flush=True); return
        if mode == "goto":
            face_goal()
            ok, L = drive_step()
            print(f"goto e={berr():.0f} F={L[0]:.2f} ok={ok}", flush=True)
            if not ok:
                # blocked: choose wall-follow side: turn toward more open of left/right
                mode = "follow"
                # beams: 4 and 12 are the two sides
                wall_side = 4 if L[4] < L[12] else 12
                print("follow, wall on beam", wall_side, flush=True)
        else:
            # wall follow keeping wall on wall_side; if goal dir clear-ish, resume
            L = rob.lidar()
            e = berr()
            if abs(e) < 30 and L[0] > 0.8:
                mode="goto"; continue
            # steer: keep wall_side dist ~0.2
            d = L[wall_side]
            if d < 0: d = 2.0
            front = L[0]
            if 0 < front < FRONT_STOP:
                # turn away from wall side by ~90
                # rotating: use lidar to turn until front clear
                s = 25 if wall_side==4 else -25
                rob.motors(-s,s) if wall_side==4 else rob.motors(s,-s)
                # turn away from wall: if wall on 4, turn so front moves toward 12 side
                for _ in range(60):
                    Lf = rob.lidar()
                    if Lf[0] > 0.5: break
                    time.sleep(0.1)
                rob.motors(0,0)
            else:
                corr = (d - 0.2)*40
                corr = max(-8, min(8, corr))
                if wall_side == 4:
                    rob.motors(12+corr, 12-corr)
                else:
                    rob.motors(12-corr, 12+corr)
                time.sleep(0.3)
                rob.motors(0,0)
            print(f"follow e={e:.0f} F={L[0]:.2f} d={d:.2f}", flush=True)
    print("timeout", flush=True)

if __name__=="__main__":
    main()
