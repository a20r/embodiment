import nav, math, time, collections, json

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

L(f"=== ctrl7 RH-wall-follower start ({nav.x:.2f},{nav.y:.2f})")
t0=time.time(); lastsave=0; lastlog=0
# rightmost-first relative sector order (right-hand rule):
ORDER=[12,13,14,15,0,1,2,3,4,5,6,7,8]
while time.time()-t0<3300:
    nav.upd()
    if nav.GOAL: L("GOAL!!!"); break
    s=nav.get_scan()
    visits[cell(nav.x,nav.y)]+=1
    hd=math.degrees(nav.H)
    pick=None
    for i in ORDER:
        cone=min(s[(i-1)%16], s[i], s[(i+1)%16])
        if s[i]>0.40 and cone>0.30:
            pick=i; break
    if pick is None:
        for i in ORDER:
            if s[i]>0.35 and min(s[(i-1)%16],s[i],s[(i+1)%16])>0.25:
                pick=i; break
    if pick is None:
        L(f"boxed ({nav.x:.2f},{nav.y:.2f}) scan={','.join('%.2f'%v for v in s)}")
        nav.drive(-70,-70); time.sleep(0.8); nav.drive(0,0)
        continue
    tdir=(hd+22.5*pick)%360
    if time.time()-lastlog>2:
        lastlog=time.time()
        L(f"RH pos=({nav.x:.2f},{nav.y:.2f}) h={hd:.0f} pick{pick} abs={tdir:.0f} d={s[pick]:.2f} d0={nav.p0.read()}")
    if pick!=0: nav.turn_to(tdir)
    r=nav.step(dist=0.32)
    if r=="goal": L("GOAL!!!"); break
    if time.time()-lastsave>30: lastsave=time.time(); savev()
    time.sleep(0.04)
nav.drive(0,0); savev(); L("ctrl7 exit")
