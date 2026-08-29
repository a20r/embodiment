import sys; sys.path.insert(0,"/bot/src")
sys.path.insert(0,"/bot/src")
from lib import write_port, read_float
import json,re,math,time

def fit_and_guide():
    rx=[]
    for line in open("/bot/src/rx.log"):
        m=re.match(r"([\d.]+) RX: (\{.*\})",line.strip())
        if not m: continue
        try: d=json.loads(m.group(2))
        except: continue
        if d.get("who")=="B" and "x" in d:
            t=float(m.group(1))
            if t>1788042605:
                rx.append((t,float(d["x"])*1000,float(d["y"])*1000,float(d["d5"])))
    rx=rx[-150:]
    if len(rx)<20: return None
    best=None
    for cx in range(-8000,8001,200):
        for cy in range(-8000,8001,200):
            ks=[]
            for _,bx,by,s in rx:
                D=math.hypot(bx+cx,by+cy)
                ks.append(s*(D+100))
            k=sum(ks)/len(ks)
            err=sum((s-k/(math.hypot(bx+cx,by+cy)+100))**2 for _,bx,by,s in rx)
            if best is None or err<best[0]: best=(err,cx,cy,k)
    err,cx,cy,k=best
    t,bx,by,s=rx[-1]
    vx,vy=bx+cx,by+cy
    dist=math.hypot(vx,vy)
    brg=math.degrees(math.atan2(-vx,-vy))%360
    return dist,brg,err/len(rx)

while True:
    r=fit_and_guide()
    if r:
        dist,brg,res=r
        msg=dict(who="A",msg=f"B: NAV: from your last reported pos, I am at compass bearing {brg:.0f} deg, dist {dist/1000:.1f}m. Drive that bearing (avoid walls). I am parked. res={res:.4f}")
        write_port("d0", json.dumps(msg))
        print(time.strftime("%H:%M:%S"), msg["msg"], flush=True)
    time.sleep(15)
