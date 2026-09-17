import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/meet.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
R.stop()
msgs=[
 'B-TO-A: I HOLD POSITION. DO YOU SEE ME IN YOUR LIDAR? REPLY SEE OR NOTSEE.',
 'B-TO-A: I SEE NO MOVING OBJECT YET. d11=0.91. KEEP COMING.',
 'B-TO-A: WHEN YOU SEE ME OR ARRIVE, SEND READY. THEN WE PLAN GOAL SEARCH.',
]
i=0
scans=[]
while True:
    m=R.rx()
    if m:
        L('RX %s'%m.replace('\n',' | '))
    try:
        R.tx(msgs[i%len(msgs)])
        i+=1
    except Exception:
        pass
    l=R.lidar()
    if len(l)==16:
        scans.append(l)
        if len(scans)>10:
            scans.pop(0)
        if len(scans)==10:
            base=scans[0]
            ch=[(k,round(l[k],2)) for k in range(16) if base[k]>0 and l[k]>0 and abs(l[k]-base[k])>0.15]
            if ch:
                L('LIDARCHANGE %s'%ch)
    st=R.status_d()
    if st.get('here')=='1' or st.get('goal','0') not in ('0',''):
        L('FLAG %s'%json.dumps(st))
    if i%10==0:
        ll=','.join('%.1f'%x for x in l) if len(l)==16 else 'NA'
        L('MEET i=%d d11=%.3f d0=%s d5=%s txstate=%s lidar=%s'%(i,R.fget('d11'),R.get('d0'),R.get('d5'),R.txstate() if hasattr(R,'txstate') else '?',ll))
    time.sleep(0.5)
