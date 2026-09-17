import time, sys, math, traceback
sys.path.insert(0,"/bot/src")
from robot import *
K=0.0004
ppl=ppr=None
def havg(n=3):
    vals=[heading() for _ in range(n)]
    s=sum(math.sin(math.radians(v)) for v in vals)/n
    c=sum(math.cos(math.radians(v)) for v in vals)/n
    return math.degrees(math.atan2(s,c))%360
x=y=0.0
th0=havg(5)
n=0
LOG=open("/bot/src/rx.log","a",buffering=1)
KB=[
 "A BOOT TIPS: d1=LEFT motor d7=RIGHT motor, + = forward, int cmds, 0=stop",
 "A BOOT TIPS: d9=left encoder d6=right encoder (ticks, ~0.4mm each)",
 "A BOOT TIPS: d2=16 lidar beams, beam8=forward, 22.5deg apart, range~1.5m, -1=open",
 "A BOOT TIPS: d4=compass deg noisy. d0=bump flag. d5=1 while driving. d11=range to other robot",
 "A BOOT TIPS: d3=tick goal here. goal=1 goal visible, here=1 standing on goal",
 "A PLAN: world=tight maze. explore randomly. report goal/here in every ping. B searching too",
 "A PLAN: if you get here=1 stay on goal, send 'A HERE' every 2s. B will home on your d11 range",
 "A PLAN: if B reports here=1, B is on goal: home on d11 range to B (minimize d11)",
]
while True:
    try:
        n+=1
        a=rd("d9"); b=rd("d6")
        el=float(a) if a else 0.0; er=float(b) if b else 0.0
        if ppl is not None:
            ds=(el-ppl)*K
            x+=ds*math.cos(math.radians(th0)); y+=ds*math.sin(math.radians(th0))
        ppl,ppr=el,er
        s3=rd("d3",timeout=0.25) or ""
        g=1 if "goal=1" in s3 else 0
        hr=1 if "here=1" in s3 else 0
        wr("d8","B PING %.2f %.2f %.0f n=%d goal=%d here=%d"%(x,y,((havg()-th0+540)%360-180),n,g,hr))
        if n%10==0: wr("d8",KB[(n//10)%len(KB)])
        r=rd("d10",timeout=0.35)
        if r: LOG.write("%.1f RX %s\n"%(time.time(),r))
        if n%25==0: LOG.write("%.1f POSE x=%.2f y=%.2f d11=%s %s\n"%(time.time(),x,y,rd("d11"),s3))
    except Exception:
        LOG.write("%.1f ERR\n"%time.time())
        time.sleep(0.5)
