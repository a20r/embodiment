from ctl import *
import json, math, sys
TPM=1960.0
BEAM=22.5
class Odo:
    def __init__(s):
        s.x=0; s.y=0; e=enc(); s.ea,s.eb=e; s.th=heading(5)
    def update(s):
        e=enc(); h=heading(2)
        if None in e or h is None: return
        da=e[0]-s.ea; db=e[1]-s.eb; s.ea,s.eb=e
        d=(da+db)/2/TPM
        th=math.radians((h+s.th)/2 if abs(wrap(h-s.th))<30 else h)
        s.x+=d*math.cos(th); s.y+=d*math.sin(th); s.th=h
odo=Odo()
samples=[]  # (x,y,sig)
visits={}
LOG=open('/bot/src/log.txt','a')
def log(msg):
    LOG.write(msg+'\n'); LOG.flush()
def sig(n=4):
    v=[]
    for _ in range(n):
        s=rd('d11',0.3)
        if s: v.append(float(s))
    return sum(v)/len(v) if v else None
def gradient():
    # least squares plane over recent samples near current position
    pts=[p for p in samples[-40:]]
    if len(pts)<4: return None
    mx=sum(p[0] for p in pts)/len(pts); my=sum(p[1] for p in pts)/len(pts); ms=sum(p[2] for p in pts)/len(pts)
    sxx=sum((p[0]-mx)**2 for p in pts); syy=sum((p[1]-my)**2 for p in pts); sxy=sum((p[0]-mx)*(p[1]-my) for p in pts)
    sxs=sum((p[0]-mx)*(p[2]-ms) for p in pts); sys_=sum((p[1]-my)*(p[2]-ms) for p in pts)
    det=sxx*syy-sxy*sxy
    if abs(det)<1e-6: return None
    gx=(syy*sxs-sxy*sys_)/det; gy=(sxx*sys_-sxy*sxs)/det
    return math.degrees(math.atan2(gy,gx))
def cell(x,y): return (round(x/0.5),round(y/0.5))
def step():
    odo.update()
    r=ranges()
    s=sig()
    while r is None or s is None:
        r=ranges(); s=sig()
    samples.append((odo.x,odo.y,s))
    visits[cell(odo.x,odo.y)]=visits.get(cell(odo.x,odo.y),0)+1
    g=gradient()
    st=rd('d3'); msg=rd('d10',0.2)
    log(f"POS x={odo.x:.2f} y={odo.y:.2f} th={odo.th:.0f} sig={s:.3f} grad={g if g is None else round(g)} {st} rx={msg!r}")
    log("  r="+",".join(f"{v:.2f}" for v in r))
    if st and ('here=0' not in st or 'goal=0' not in st):
        log("!!! STATUS CHANGED: "+st)
    # candidate beams
    best=None; bestscore=-1e9
    for i in range(16):
        if r[i]<0: continue
        wa=(odo.th+i*BEAM)%360
        # look-ahead clearance: also neighbors
        nb=min(r[i], r[(i+1)%16] if r[(i+1)%16]>0 else 9, r[(i-1)%16] if r[(i-1)%16]>0 else 9)
        if r[i]<0.55: continue
        dist=min(0.5, r[i]-0.3)
        nx=odo.x+dist*math.cos(math.radians(wa)); ny=odo.y+dist*math.sin(math.radians(wa))
        score=0
        if g is not None: score+=math.cos(math.radians(wa-g))*2.0
        score+=min(r[i],2.0)*0.3
        score-=visits.get(cell(nx,ny),0)*1.0
        if i==8: score-=0.5
        if score>bestscore: bestscore=score; best=(i,wa,dist)
    if best is None:
        log("  no candidates; turning around"); spin_by(180); return
    i,wa,dist=best
    log(f"  -> beam {i} heading {wa:.0f} dist {dist:.2f} score {bestscore:.2f}")
    if i!=0: spin_to(wa)
    res=drive(150, ticks=int(dist*TPM), minfront=0.28)
    odo.update()
    log(f"  drove {res}")
if __name__=='__main__':
    n=int(sys.argv[1]) if len(sys.argv)>1 else 30
    for k in range(n):
        try: step()
        except Exception as ex: log("ERR "+repr(ex)); stop()
    stop(); log("DONE")
