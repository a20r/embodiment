from ctl import Ctl, angdiff
import time
b=Ctl(); time.sleep(0.3)
def unstick():
    for it in range(12):
        s=b.fresh() if hasattr(b,'fresh') else None
        s=[x if x>0 else 0.05 for x in b.scan()]
        mn=min(s); mi=s.index(mn)
        b.p[7].poll()
        if mn>0.16: break
        # away direction = beam mi+8; decide to drive forward or backward with arc
        # if closest is on left side (beams 1..7): arc right-forward if front clear else back-right
        h=b.heading()
        if mi in (0,1,15):  # front contact: back up
            b.wr(4,'25'); b.wr(5,'25'); time.sleep(0.5)
        elif mi in (7,8,9): # rear: forward
            b.wr(4,'-25'); b.wr(5,'-25'); time.sleep(0.5)
        elif 2<=mi<=6:  # left side: rotate slightly right then forward? better: rotate so contact moves to rear-left, then forward-right arc
            b.wr(4,'-28'); b.wr(5,'-16'); time.sleep(0.45)  # arc turning right? l more negative => l faster => turns toward right(-w): r-l=12 => w>0?? 
        else: # right side
            b.wr(4,'-16'); b.wr(5,'-28'); time.sleep(0.45)
        b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
        print('unstick it',it,'mi',mi,'mn',round(mn,2))
    s=[x if x>0 else 0.05 for x in b.scan()]
    print('final scan',[round(x,2) for x in s])
unstick()
