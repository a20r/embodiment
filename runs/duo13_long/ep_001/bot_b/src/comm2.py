import time, sys, math
sys.path.insert(0,"/bot/src")
from robot import *
# Odometry from NOW: x,y in meters (K=0.0004 m/tick guess), th from compass at start
K=0.0004
TRACK=0.174  # deg per tick-diff
el=er=0.0
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
rdenc()
x=y=0.0
th0=havg(5)
n=0
LOG=open("/bot/src/rx.log","a",buffering=1)
while True:
    n+=1
    wr("d8","B PING %.2f %.2f %.0f n=%d"%(x,y,((havg()-th0+540)%360-180),n))
    r=rd("d10",timeout=0.35)
    if r:
        LOG.write("%.1f RX %s\n"%(time.time(),r))
    rdenc()
    dl=(el)*K; dr=(er)*K
    ds=(dl+dr)/2; dth=(er-el)*TRACK
    # integrate: el,er are cumulative? no - I reset el,er each loop via absolute reads... fix: store prev
    prev=(el,er)
    # do integration with previous values
    try:
        ds=(el-ppl)*K; dr=(er-ppr)*K
        dth=((er-ppr)-(el-ppl))*TRACK
    except NameError:
        ds=dr=dth=0
    ppl,ppr=el,er
    x+=ds*math.cos(math.radians(th0)); y+=ds*math.sin(math.radians(th0))
    if n%20==0:
        LOG.write("%.1f POSE x=%.2f y=%.2f d11=%s d3=%s\n"%(time.time(),x,y,rd("d11"),rd("d3")))
