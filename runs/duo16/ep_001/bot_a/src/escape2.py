import sys; sys.path.insert(0,'/bot/src')
import drv, math, time

def close_mean(pts):
    """mean range of nearby points (robot still wedged -> small; escaped -> large)"""
    rs=[math.hypot(x,y) for x,y,z in pts if -0.03<z<0.4]
    near=[r for r in rs if r<0.35]
    return sum(near)/len(near) if near else 0.5

def openness(pts, az_lo, az_hi):
    m=0
    for x,y,z in pts:
        if -0.03<z<0.4:
            az=math.degrees(math.atan2(x,y))
            if az_lo<=az<=az_hi:
                m=max(m,math.hypot(x,y))
    return m

def try_escape(log=print):
    base=drv.scan(); m0=close_mean(base)
    log(f"start close_mean={m0:.3f} h={drv.heading()}")
    best=(None,-1)
    for az in [-100,-60,-20,+20,+60]:
        # rotate so that cloud-az 'az' comes to 0: heading += az
        cur=drv.scan(); b,_=drv.best_bearing(cur,fov=180,sector=5)
        # measure current offset of target sector
        # simply rotate by +az using time estimate ~ (az/10)*0.35s at (3,-3)
        if az>0: drv.spin((3,-3), abs(az)*0.045)
        else:    drv.spin((-3,3), abs(az)*0.045)
        for k in range(4):
            drv._w('d1','2.2'); drv._w('d7','2.2'); time.sleep(0.7)
            drv._w('d1','1.6'); drv._w('d7','2.6'); time.sleep(0.25)  # arc wiggle
            drv._w('d1','2.6'); drv._w('d7','1.6'); time.sleep(0.25)
            drv.stop(); time.sleep(0.05)
        drv.stop()
        s=drv.scan(); m=close_mean(s)
        log(f"  az={az:+4d}: close_mean={m:.3f} h={drv.heading()}")
        if m>best[1]: best=(az,m)
        if m>0.42:
            log("  escaped via az",az); return True
    # go back toward best
    log("best was",best)
    return best[1]>m0+0.05

if __name__=="__main__":
    try_escape()
