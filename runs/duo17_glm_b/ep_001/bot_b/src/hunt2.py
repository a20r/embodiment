import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/hunt2.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

def sig(n=35,dt=0.05):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0

def turn_to(tgt,tol=5,timeout=4):
    t0=time.time()
    while time.time()-t0<timeout:
        err=((tgt-R.heading()+180)%360)-180
        if abs(err)<=tol: R.stop(); return
        s=24 if err>0 else -24
        R.motors(s,-s); time.sleep(0.07)
    R.stop()

def drive_t(secs,spd=24):
    R.motors(spd,spd); time.sleep(secs); R.stop()

def front_min():
    l=R.lidar()
    if len(l)!=16: return 9
    return min([x for x in (l[15],l[0],l[1]) if x>0] or [9])

BEACON=['B-TO-A: I AM NOW DRIVING TOWARD YOU. KEEP TXING, DO NOT MOVE.',
        'B-TO-A: HOMING ON YOUR CARRIER. WILL STOP WHEN d11>0.97.']
def beacon():
    import time as _t
    try:
        R.tx(BEACON[int(_t.time())%2])
    except Exception: pass

def main():
    R.stop()
    L('HUNT2 start d11=%.3f h=%.1f'%(sig(),R.heading()))
    best=sig()
    h=R.heading()
    # initial 8-way scan
    scores=[]
    h0=R.heading()
    for i in range(0,360,45):
        tgt=(h0+i)%360
        turn_to(tgt); time.sleep(0.3)
        s=sig(20)
        scores.append((s,tgt))
        L('SCAN tgt=%d s=%.3f'%(tgt,s))
    scores.sort(reverse=True)
    best=scores[0][0]; cur=scores[0][1]
    L('BEST heading %d s=%.3f'%(cur,best))
    fails=0; sign=1; it=0
    last_bc=0
    while True:
        it+=1
        turn_to(cur); 
        drive_t(1.3)
        s=sig()
        L('IT%d drive h=%d s=%.3f best=%.3f'%(it,int(R.heading()),s,best))
        if time.time()-last_bc>25:
            beacon(); last_bc=time.time()
        st=R.status_d()
        if st.get('here')=='1' or st.get('goal','0') not in ('0',''):
            L('FLAG %s'%json.dumps(st)); break
        f=front_min()
        if f<0.2:
            L('obstacle front=%.2f'%f)
        if s>best+0.004:
            best=s; fails=0
            if s>0.97:
                # close! tiny moves
                drive_t(0.5,18)
                s=sig(25)
                L('CLOSE move s=%.3f'%s)
                if s<=best-0.01:
                    drive_t(-0.0)
                    R.motors(-18,-18); time.sleep(0.6); R.stop()
                best=max(best,s)
            continue
        elif s>best-0.006:
            fails+=1
            if fails>4:
                fails=0
                cur=(cur+sign*60)%360; sign*=-1
                L('plateau, trying heading %d'%cur)
            continue
        else:
            # worse: back up and turn
            R.motors(-22,-22); time.sleep(1.0); R.stop()
            cur=(cur+sign*60)%360; sign*=-1
            fails=0
            L('worse, new heading %d'%cur)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('HUNT2 end d11=%.3f'%sig())
