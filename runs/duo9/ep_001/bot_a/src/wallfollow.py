import time, math, json, sys, random
sys.path.insert(0,'/bot/src')
from bot import *
M_PER_COUNT=0.00078
LOG=open('/memory/maplog.jsonl','a',buffering=1)
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n')
class Odo:
    def __init__(s):
        s.x=0.0; s.y=0.0; s.el,s.er=enc()
    def update(s,h):
        el,er=enc()
        d=((el-s.el)+(er-s.er))/2.0*M_PER_COUNT
        s.el,s.er=el,er
        th=math.radians(h)
        s.x+=d*math.cos(th); s.y+=d*math.sin(th)
odo=Odo()
def mn(vals): 
    v=[x for x in vals if x>=0]
    return min(v) if v else 9.0
last_tx=[0]
def housekeeping(L):
    h=heading(); odo.update(h)
    st=status()
    now=time.time()
    if st.get('here') or st.get('goal'):
        speed(0,0); tx('AT_GOAL'); log(note='HERE',st=st)
        return True
    if now-last_tx[0]>2:
        tx('POS %.2f %.2f'%(odo.x,odo.y)); last_tx[0]=now
    return False
tlog=[0]
def maybe_log(L,extra=None):
    now=time.time()
    if now-tlog[0]>0.8:
        log(x=round(odo.x,2),y=round(odo.y,2),h=heading(),L=L,d5=rd('d5'),n=extra)
        tlog[0]=now
def turn_left_until_clear(maxt=8):
    # l<r : features shift to higher indices
    t0=time.time()
    while time.time()-t0<maxt:
        speed(-22,22); time.sleep(0.25); speed(0,0); time.sleep(0.15)
        L=lidar()
        if housekeeping(L): return 'goal'
        maybe_log(L,'turnL')
        if mn([L[0],L[1],L[15]])>0.5: return 'ok'
    return 'timeout'
def escape():
    log(note='escape')
    speed(-30,-30); time.sleep(1.2); speed(0,0); time.sleep(0.2)
    for _ in range(random.randint(2,5)):
        speed(-22,22); time.sleep(0.3); speed(0,0); time.sleep(0.1)
    speed(0,0)
hist=[]
def stuck_check(L):
    now=time.time()
    hist.append((now,L))
    while hist and now-hist[0][0]>6: hist.pop(0)
    if hist and now-hist[0][0]>5.5:
        old=hist[0][1]
        diff=sum(abs(a-b) for a,b in zip(old,L) if a>=0 and b>=0)
        if diff<0.5:
            hist.clear(); return True
    return False
def main():
    RT=0.30  # right wall target
    while True:
        L=lidar()
        if housekeeping(L): time.sleep(0.5); continue
        maybe_log(L)
        if stuck_check(L):
            escape(); continue
        f=mn([L[0],L[1],L[15]])
        r=mn([L[3],L[4],L[5]])
        rfront=mn([L[2],L[3]])
        if f<0.32:
            speed(0,0)
            if turn_left_until_clear()=='goal': continue
            continue
        # steering: keep right wall at RT
        err=r-RT   # >0: too far from wall -> turn right (l>r)
        if r>0.9 and rfront>0.6:
            # opening on right: arc right
            speed(35,12)
        else:
            corr=max(-12,min(12,err*45))
            base=35 if f>0.6 else 22
            speed(base+corr,base-corr)
        time.sleep(0.09)
if __name__=='__main__':
    try: main()
    finally: speed(0,0)
