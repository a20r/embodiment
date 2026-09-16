import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
K=0.0004
el=er=0.0; ppl=None; ppr=None
def rdenc():
    global el,er
    a=rd("d9"); b=rd("d6")
    if a: el=float(a)
    if b: er=float(b)
def havg(n=3):
    vals=[heading() for _ in range(n)]
    s=sum(math.sin(math.radians(v)) for v in vals)/n
    c=sum(math.cos(math.radians(v)) for v in vals)/n
    return math.degrees(math.atan2(s,c))%360
x=y=0.0
th0=havg(5)
n=0
LOG=open("/bot/src/rx.log","a",buffering=1)
while True:
    n+=1
    rdenc()
    if ppl is not None:
        ds=(el-ppl)*K
        x+=ds*math.cos(math.radians(th0)); y+=ds*math.sin(math.radians(th0))
    ppl,ppr=el,er
    s3=rd("d3",timeout=0.3) or ""
    g = 1 if "goal=1" in s3 else 0
    hr = 1 if "here=1" in s3 else 0
    msg="B PING %.2f %.2f %.0f n=%d goal=%d here=%d"%(x,y,((havg()-th0+540)%360-180),n,g,hr)
    wr("d8",msg)
    if n%20==0: wr("d8","B MSG please go to goal and report position. B is exploring.")
    r=rd("d10",timeout=0.35)
    if r: LOG.write("%.1f RX %s\n"%(time.time(),r))
    if n%20==0: LOG.write("%.1f POSE x=%.2f y=%.2f d11=%s %s\n"%(time.time(),x,y,rd("d11"),s3))
