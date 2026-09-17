import time, math
def rd(p):
    with open('/dev/robot/'+p) as f: return f.readline().strip()
def num(p):
    try: return float(rd(p))
    except: return None
x=y=0.0
pl=pr=None
last=0
SCALE=785.0
while True:
    l=num('d7'); r=num('d8'); h=num('d1')
    if l is not None and r is not None and h is not None:
        if pl is not None:
            dl=l-pl; dr=r-pr
            if 0<=dl<5000 and 0<=dr<5000:
                d=(dl+dr)/2.0/SCALE
                x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
        pl,pr=l,r
        now=time.time()
        if now-last>5:
            last=now
            with open('/tmp/track.log','a') as f:
                f.write('%.0f %.2f %.2f %.1f\n'%(now,x,y,h))
    time.sleep(0.25)
