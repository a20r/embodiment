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
        ds=(el-ppl)*K; dth=((er-ppr)-(el-ppl))*0.174
        x+=ds*math.cos(math.radians(th0)); y+=ds*math.sin(math.radians(th0))
    ppl,ppr=el,er
    wr("d8","B PING %.2f %.2f %.0f n=%d"%(x,y,((havg()-th0+540)%360-180),n))
    if n%15==0: wr("d8","B GOAL? WHERE GOAL")
    r=rd("d10",timeout=0.35)
    if r: LOG.write("%.1f RX %s\n"%(time.time(),r))
    if n%20==0: LOG.write("%.1f POSE x=%.2f y=%.2f d11=%s d3=%s\n"%(time.time(),x,y,rd("d11"),rd("d3")))
