import sys, time, math, json
sys.path.insert(0,'/bot/src')
from robot2 import *

POSE={'x':0.0,'y':0.0,'h':heading()}
_last=[odom()]
def upd():
    lo,ro=odom()
    pl,pr=_last[0]
    _last[0]=(lo,ro)
    d=((lo-pl)+(ro-pr))/2*0.002
    h=heading()
    POSE['x']+=d*math.cos(math.radians(h))
    POSE['y']+=d*math.sin(math.radians(h))
    POSE['h']=h
    return POSE

def best_dir(l):
    best=None; bs=-1e9
    for k in range(16):
        w=[l[(k+j)%16] for j in (-1,0,1)]
        w=[x for x in w if x>0]
        if not w: continue
        m=min(w)
        if m<0.30: continue
        off=min(k,16-k)
        s=m-0.05*off
        if s>bs: bs=s; best=k
    return best

LOG=open('/memory/explore.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000, s)); LOG.flush()

def main():
    t0=time.time()
    lastping=0; seq=0
    while True:
        st=status_d()
        if st.get('here')=='1':
            L('HERE=1 '+json.dumps(st)); break
        if st.get('goal','0') not in ('0',''):
            L('GOAL! '+json.dumps(st)); break
        msg=rx()
        if msg:
            L('RX '+msg)
            tx('ACK A')
        now=time.time()
        if now-lastping>3:
            seq+=1
            tx(f'A {seq} pos {POSE["x"]:.1f},{POSE["y"]:.1f}')
            lastping=now
            if seq%10==0:
                L('ST %s b=%.2f d0=%s d5=%s'%(json.dumps(st), battery(), get('d0'), get('d5')))
        l=lidar()
        if not l: time.sleep(0.05); continue
        k=best_dir(l)
        h=POSE['h']
        if k is None:
            motors(-28,-28); time.sleep(0.6); stop(); upd(); continue
        target=(h+22.5*k)%360
        err=((target-h+180)%360)-180
        if abs(err)>10:
            s=30 if err>0 else -30
            motors(s,-s)
        else:
            gain=max(-18,min(18,err*2.0))
            motors(26-gain,26+gain)
        upd()
        time.sleep(0.1)
    stop()

if __name__=='__main__':
    try:
        main()
    finally:
        stop()
        L('END pose %s'%json.dumps(POSE))
