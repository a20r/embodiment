import time, sys, math, traceback
sys.path.insert(0,"/bot/src")
from robot import *
K=0.0004
ppl=None; ppr=None
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
        if n%25==0: wr("d8","B MSG if you found goal, stand on it. B coming.")
        r=rd("d10",timeout=0.35)
        if r: LOG.write("%.1f RX %s\n"%(time.time(),r))
        if n%25==0: LOG.write("%.1f POSE x=%.2f y=%.2f d11=%s %s\n"%(time.time(),x,y,rd("d11"),s3))
    except Exception:
        LOG.write("%.1f ERR %s\n"%(time.time(),traceback.format_exc().replace("\n","|")[:200]))
        time.sleep(0.5)
