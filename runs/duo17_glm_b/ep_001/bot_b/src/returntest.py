import sys,time,statistics,math
sys.path.insert(0,'/bot/src')
import robot2 as R
def med(n=2):
    scans=[]
    for _ in range(40):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=n: break
        time.sleep(0.04)
    return [round(statistics.median([s[i] for s in scans]),2) for i in range(16)]
cx,cy=0.28,-1.63
tx,ty=0.0,0.0
dx,dy=tx-cx,ty-cy
dist0=math.hypot(dx,dy)
bearing=math.degrees(math.atan2(dy,dx))%360
LOG=open('/memory/returntest.log','a')
def L(s): LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
L('target dist %.2f bearing %.0f'%(dist0,bearing))
o0=R.odom()
px,py=cx,cy
t0=time.time()
while time.time()-t0<8:
    err=((bearing-R.heading()+180)%360)-180
    if abs(err)<5: break
    if err>0: R.motors(20,-20)
    else: R.motors(-20,20)
    time.sleep(0.1)
R.stop()
L('rotated to %.0f'%R.heading())
traveled=0.0
while time.time()-t0<40 and traveled<dist0-0.15:
    l=med()
    front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
    if front<0.22:
        L('blocked early at traveled %.2f front %.2f'%(traveled,front)); break
    R.motors(18,18); time.sleep(0.3); R.stop()
    o1=R.odom()
    d=((o1[0]-o0[0])+(o1[1]-o0[1]))/2*0.00066
    o0=o1
    traveled+=abs(d)
    h=math.radians(R.heading())
    px+=d*math.cos(h); py+=d*math.sin(h)
    time.sleep(0.05)
L('stopped at pose %.2f,%.2f traveled %.2f'%(px,py,traveled))
L('scan: %s'%med(4))
R.stop()
t1=time.time()
while time.time()-t1<75:
    st=R.status_d()
    d0=R.get('d0'); d5=R.get('d5')
    if d0!='0' or d5!='0' or st.get('here')!='0' or st.get('goal','0')!='0':
        L('EVENT t=%.1f d0=%s d5=%s st=%s'%(time.time()-t1,d0,d5,st))
    time.sleep(0.5)
L('observed 75s. d11=%.3f status=%s'%(R.fget('d11'),R.status()))
