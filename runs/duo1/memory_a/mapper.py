import rob, walker, time, math, json, random

def ad(x): return (x+180)%360-180

class EKF:
    def __init__(s, D0, r0):
        s.x=-D0; s.y=0.0
        s.h=-r0  # deg
        s.lastodo=rob.odo()
    def predict(s):
        o=rob.odo(); ds=(o-s.lastodo)/156.0; s.lastodo=o
        s.x+=ds*math.cos(math.radians(s.h)); s.y+=ds*math.sin(math.radians(s.h))
        return ds
    def correct(s, r):
        phi=math.degrees(math.atan2(-s.y,-s.x))
        rp=ad(phi-s.h)
        e=ad(r-rp)
        s.h=(s.h+0.6*e)%360
    def D(s): return math.hypot(s.x,s.y)

def measure_D():
    walker.turn(ad(90-walker.bng()))
    b0=walker.bng(); o0=rob.odo()
    rob.motors(15,15)
    while rob.odo()-o0<70:
        L=rob.lidar()
        if 0<L[0]<0.25: break
        time.sleep(0.04)
    rob.motors(0,0)
    b1=walker.bng(); ds=(rob.odo()-o0)/156.0
    db=abs(ad(b1-b0))
    if db<2: return 2.5
    D=ds*math.sin(math.radians((b0+b1)/2))/math.tan(math.radians(db))
    return max(0.3,min(3.0,abs(D)))

def main():
    D0=measure_D()
    r0=walker.bng()
    ekf=EKF(D0,r0)
    print("D0",D0,flush=True)
    f=open('/tmp/map.jsonl','w')
    t0=time.time()
    def tick(moving=True):
        ekf.predict()
        r=walker.bng()
        ekf.correct(r)
        L=rob.lidar()
        f.write(json.dumps([round(time.time()-t0,2),round(ekf.x,3),round(ekf.y,3),round(ekf.h,1),round(r,1)]+L)+"\n")
        return r,L
    def turn_tracked(delta):
        # in-place turn: reading change = -heading change... turn(+d) increases reading; heading h decreases? keep convention h-=delta
        b_before=walker.bng()
        walker.turn(delta)
        b_after=walker.bng()
        real=ad(b_after-b_before)
        ekf.h=(ekf.h-real)%360
    while time.time()-t0<240:
        if rob.goal(): print("GOAL!!!",flush=True); break
        walker.align()
        # alignment rotations are untracked: fix by correcting with current reading assuming pos same
        r=walker.bng()
        phi=math.degrees(math.atan2(-ekf.y,-ekf.x))
        ekf.h=(phi-r)%360
        F,R,Lt,B=walker.look()
        opts=[]
        for dd,dist in ((0,F),(90,R),(-90,Lt)):
            if dist>0.38: opts.append((dd,dist))
        if not opts:
            turn_tracked(180); continue
        dd=random.choice(opts)[0]
        if dd: turn_tracked(dd)
        o0=rob.odo()
        while rob.odo()-o0<78:
            L=rob.lidar()
            if 0<L[0]<0.20: break
            r_,l_=L[4],L[12]
            c=0.0
            if 0<r_<0.45 and 0<l_<0.45: c=(r_-l_)*35
            elif 0<r_<0.28: c=(r_-0.18)*45
            elif 0<l_<0.28: c=-(l_-0.18)*45
            c=max(-7,min(7,c))
            rob.motors(14+c,14-c)
            tick()
            time.sleep(0.04)
        rob.motors(0,0)
    f.close()
    print("done",flush=True)

if __name__=="__main__":
    main()
