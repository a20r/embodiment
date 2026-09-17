import math, sys
rows=[]
for ln in open('/bot/src/map_input.csv'):
    a=ln.strip().split(',')
    if len(a)!=15: continue
    try: rows.append((int(a[1]), float(a[5]), float(a[6])))  # tick, hd, sp
    except: pass
rows.sort()
fires={472527:'lap2->3',663669:'lap3->4',687778:'lap4->5'}
x=y=0.0; prev=None
trail=[]; firepos={}
seglen=0.0; totlen=0.0; nseg=0
hd_prev=None
for tick,hd,sp in rows:
    if abs(sp)>2.5: sp=math.copysign(2.5,sp)
    if hd_prev is not None and abs((hd-hd_prev+180)%360-180)>60: hd=hd_prev
    hd_prev=hd
    if prev is None:
        prev=tick; trail.append((x,y,tick)); continue
    dt=(tick-prev)/100.0
    prev=tick
    if dt<=0: continue
    if dt>2.0:  # gap: translation-chain new segment
        nseg+=1; seglen=0.0; trail.append((x,y,tick)); continue
    r=math.radians(hd)
    step=sp*dt
    x+=step*math.cos(r); y+=step*math.sin(r)
    seglen+=step; totlen+=step
    trail.append((x,y,tick))
    if tick in fires: firepos[fires[tick]]=(x,y)
print('rows',len(rows),'segments',nseg+1,'total len %.1f m'%totlen)
print('start', (round(trail[0][0],1),round(trail[0][1],1)), 'end', (round(trail[-1][0],1),round(trail[-1][1],1)))
for k,v in sorted(firepos.items()): print('FIRE',k,'at (%.1f, %.1f)'%v)
xs=[p[0] for p in trail]; ys=[p[1] for p in trail]
print('bbox x [%.1f, %.1f] y [%.1f, %.1f]'%(min(xs),max(xs),min(ys),max(ys)))
import pickle
pickle.dump(trail, open('/bot/src/world_trail.pkl','wb'))
