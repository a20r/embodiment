import nav, math, time, random, collections, json, os

log=open("/memory/run.log","a",buffering=1)
def L(*a): print(time.strftime("%H:%M:%S"),*a,file=log)

CELL=0.35
visits=collections.Counter()
try:
    for k,v in json.load(open("/memory/visits.json")).items():
        a,b=k.split(","); visits[(int(a),int(b))]=v
except Exception: pass

def cell(px,py): return (round(px/CELL), round(py/CELL))
def savev():
    json.dump({f"{a},{b}":v for (a,b),v in visits.items()}, open("/memory/visits.json","w"))
    nav.save()

wrap=nav.wrap
L(f"=== ctrl6 start pose=({nav.x:.2f},{nav.y:.2f}) cells={len(visits)}")
t0=time.time(); lastdir=0.0; lastsave=0; lastlog=0
dirpen=collections.defaultdict(float)
while time.time()-t0<3300:
    nav.upd()
    if nav.GOAL: L("GOAL!!!"); break
    s=nav.get_scan()
    visits[cell(nav.x,nav.y)]+=1
    hd=math.degrees(nav.H)
    best=None; bestsc=-99; bestray=0
    for i in range(16):
        cone=min(s[(i-1)%16], s[i], s[(i+1)%16])
        if cone<0.28: continue
        absang=(hd+22.5*i)%360
        sect=int(absang//22.5)%16
        ang=math.radians(absang)
        reach=min(s[i]-0.2,0.5)
        v=visits[cell(nav.x+reach*math.cos(ang), nav.y+reach*math.sin(ang))]
        v2=visits[cell(nav.x+2.5*reach*math.cos(ang), nav.y+2.5*reach*math.sin(ang))] if s[i]>0.8 else 5
        sc=min(cone,1.0)-0.5*v-0.15*min(v2,6)-dirpen[sect]
        if s[i]>1.4:
            fx,fy=nav.x+1.2*math.cos(ang), nav.y+1.2*math.sin(ang)
            sc += 1.3 if visits[cell(fx,fy)]==0 else 0.15
        if abs(wrap(22.5*i-180))<34: sc-=0.8
        if abs(wrap(absang-lastdir))<25: sc+=0.3
        sc+=random.uniform(0,0.05)
        if sc>bestsc: bestsc,best,bestray=sc,i,s[i]
    for k in dirpen: dirpen[k]*=0.6
    if best is None:
        L(f"boxed ({nav.x:.2f},{nav.y:.2f})"); nav.drive(-70,-70); time.sleep(0.8); nav.drive(0,0)
        continue
    tdir=(hd+22.5*best)%360
    lastdir=tdir
    if time.time()-lastlog>2:
        lastlog=time.time()
        L(f"pos=({nav.x:.2f},{nav.y:.2f}) h={hd:.0f} ray{best} abs={tdir:.0f} sc={bestsc:.2f} d={bestray:.2f} v={visits[cell(nav.x,nav.y)]} d0={nav.p0.read()}")
    if best!=0: nav.turn_to(tdir)
    r=nav.step(dist=min(max(0.3,bestray-0.3),0.8))
    if r=="goal": L("GOAL!!!"); break
    if r in ("bump","blocked"):
        sect=int(tdir//22.5)%16
        dirpen[sect]+=1.2
        ang=math.radians(tdir)
        visits[cell(nav.x+0.25*math.cos(ang),nav.y+0.25*math.sin(ang))]+=2
    if time.time()-lastsave>30:
        lastsave=time.time(); savev()
    time.sleep(0.05)
nav.drive(0,0); savev(); L("ctrl6 exit")
