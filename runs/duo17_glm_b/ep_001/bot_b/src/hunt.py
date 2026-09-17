import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/hunt.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
def sig(n=6):
    vs=[R.fget('d11') for _ in range(n)]
    vs=[v for v in vs if v>0]
    return statistics.median(vs) if vs else 0.0
def turn_to(tgt,tol=4,timeout=5):
    t0=time.time()
    while time.time()-t0<timeout:
        err=((tgt-R.heading()+180)%360)-180
        if abs(err)<=tol: R.stop(); return
        s=26 if err>0 else -26
        R.motors(s,-s); time.sleep(0.07)
    R.stop()
# phase 1: scan 360 for best signal heading
def scan(step=30):
    best=(-1,0)
    h0=R.heading()
    for i in range(0,360,step):
        tgt=(h0+i)%360
        turn_to(tgt,tol=6,timeout=3)
        time.sleep(0.25)
        s=sig()
        L('SCAN h=%.0f sig=%.3f'%(R.heading(),s))
        if s>best[0]: best=(s,R.heading())
    return best
best=scan()
L('BEST heading %.0f sig %.3f'%(best[1],best[0]))
# phase 2: iterate drive-toward-best
cur=best[1]
for it in range(30):
    st=R.status_d()
    if st.get('here')=='1' or st.get('goal','0') not in ('0',''):
        L('FLAG %s'%json.dumps(st)); break
    turn_to(cur,tol=5,timeout=4)
    o0=R.odom()
    R.motors(24,24); time.sleep(1.4); R.stop()
    s=sig()
    l=R.lidar()
    L('MOVE it=%d h=%.0f sig=%.3f front=%.2f d0=%s d5=%s'%(it,R.heading(),s,min([x for x in l[14:17%16] if x>0] or [9]) if l else 9,R.get('d0'),R.get('d5')))
    if s<best[0]-0.01:
        # signal dropped: rescan
        nb=scan(45)
        L('RESCAN best %.3f at %.0f'%(nb[0],nb[1]))
        if nb[0]>best[0]: best=nb
        cur=best[1]
    else:
        best=max(best,(s,cur))
        cur=R.heading()
        if s>0.97:
            L('SIGNAL SATURATED %.3f — LOOK AROUND'%s)
            time.sleep(0.5)
R.stop()
L('HUNT end sig=%.3f'%sig())
