import math
def integ(path, fires):
    rows=[]
    for ln in open(path):
        a=ln.strip().split(',')
        if len(a)<14: continue
        try: rows.append((int(a[1]), float(a[5]), float(a[6])))
        except: pass
    rows.sort()
    x=y=0.0; prev=None; hp=None; out={}; trail=[]
    for tick,hd,sp in rows:
        if abs(sp)>2.5: sp=math.copysign(2.5,sp)
        if hp is not None and abs((hd-hp+180)%360-180)>60: hd=hp
        hp=hd
        for fk in fires:
            if abs(tick-fk)<=40 and fk not in out: out[fk]=(x,y)
        if prev is None: prev=tick; trail.append((x,y,tick)); continue
        dt=(tick-prev)/100.0; prev=tick
        if dt<=0: continue
        if dt>2.0: trail.append((x,y,tick)); continue
        r=math.radians(hd); step=sp*dt
        x+=step*math.cos(r); y+=step*math.sin(r)
        trail.append((x,y,tick))
    return out,trail
o8,t8=integ('/bot/src/auto8.csv',{434665:'F12',472527:'F23'})
# overlap: auto9 world starts at 0 at its first row tick T0
import pickle
trail9,_=pickle.load(open('/bot/src/world_trail.pkl','rb'))
T0=min(t for x,y,t in trail9 if t>400000)
p8=None
for x,y,t in t8:
    if abs(t-T0)<=20: p8=(x,y); break
print('auto8 fires:',{k:(round(v[0],1),round(v[1],1)) for k,v in o8.items()})
print('auto8 pos at T0=%d: %s'%(T0, None if p8 is None else (round(p8[0],2),round(p8[1],2))))
if p8:
    dx,dy=0-p8[0], 0-p8[1]
    print('translation auto8->auto9 world: (%.1f,%.1f)'%(dx,dy))
    for k,(x,y) in o8.items():
        print(k,'in auto9 world: (%.1f, %.1f)'%(x+dx,y+dy))
print('EXPECTED zone: (7.75, -107.2)')
