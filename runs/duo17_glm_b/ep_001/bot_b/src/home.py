import sys, time, statistics, math, json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/home.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

def sig(n=8):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(0.06)
    return statistics.median(vs) if vs else 0.0

def heading(): return R.heading()

def turn(deg_err, speed=30):
    # deg_err>0 => CW (heading increase): left fwd right back
    s=abs(speed) if deg_err>0 else -abs(speed)
    R.motors(s,-s)

def turn_to(target, tol=4, timeout=6):
    t0=time.time()
    while time.time()-t0<timeout:
        err=((target-heading()+180)%360)-180
        if abs(err)<=tol:
            R.stop(); return
        turn(err, speed=28)
        time.sleep(0.08)
    R.stop()

def drive(secs, spd=26):
    R.motors(spd,spd); time.sleep(secs); R.stop()

def main():
    L('HOME start sig=%.3f h=%.1f'%(sig(),heading()))
    best=sig()
    cur_heading=heading()
    trial=0
    try_more=0
    while True:
        s0=sig()
        h=heading()
        # try current heading first
        drive(1.1)
        s1=sig()
        L('trial %d straight: %.3f -> %.3f (best %.3f) h=%.0f'%(trial,s0,s1,best,heading()))
        if s1 >= s0-0.004:
            # improving or level: continue, and record
            if s1>best:
                best=s1; L('NEW BEST %.3f'%best)
            if s1>0.9:
                L('SIGNAL VERY STRONG %.3f - SLOW SCAN'%s1)
            trial+=1
            continue
        # degraded: turn and probe angles
        turn_dirs=[50,-50,100,-100,160,-160]
        improved=False
        for d in turn_dirs:
            target=(h+d)%360
            turn_to(target, tol=5, timeout=4)
            time.sleep(0.2)
            sp=sig()
            drive(0.9)
            s2=sig()
            L('  probe %+d: sig %.3f after drive %.3f (s0 %.3f)'%(d,sp,s2,s0))
            if s2 > s0+0.002:
                improved=True
                if s2>best: best=s2; L('NEW BEST %.3f'%best)
                break
        if not improved:
            L('  no improvement; backup and re-try')
            R.motors(-25,-25); time.sleep(0.8); R.stop()
        trial+=1
        st=R.status_d()
        if st.get('here')=='1' or st.get('goal','0') not in ('0',''):
            L('STATUS FLAG! '+json.dumps(st))
        if R.fget('d0')!=0 or R.fget('d5')!=0:
            L('D0/D5 CHANGE! d0=%s d5=%s'%(R.get('d0'),R.get('d5')))
        time.sleep(0.1)

if __name__=='__main__':
    try:
        main()
    finally:
        R.stop()
        L('HOME end sig=%.3f h=%.1f'%(sig(),heading()))
