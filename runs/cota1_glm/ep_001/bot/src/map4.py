import math, pickle
rows=[]
for ln in open('/bot/src/map_input.csv'):
    a=ln.strip().split(',')
    if len(a)!=15: continue
    try: rows.append((int(a[1]), float(a[5]), float(a[6])))
    except: pass
rows.sort()
x=y=0.0; prev=None; hp=None; trail=[]; nseg=0
for tick,hd,sp in rows:
    if abs(sp)>2.5: sp=math.copysign(2.5,sp)
    if hp is not None and abs((hd-hp+180)%360-180)>60: hd=hp
    hp=hd
    if prev is None: prev=tick; trail.append((x,y,tick)); continue
    dt=(tick-prev)/100.0; prev=tick
    if dt<=0: continue
    if dt>2.0: nseg+=1; trail.append((x,y,tick)); continue
    r=math.radians(hd); step=sp*dt
    x+=step*math.cos(r); y+=step*math.sin(r)
    trail.append((x,y,tick))
print('rows',len(rows),'segs',nseg+1,'end (%.1f,%.1f)'%(x,y),'lasttick',trail[-1][2])
pickle.dump(trail, open('/bot/src/world_trail.pkl','wb'))
def render(pts,CX=64,CY=56,tag=''):
    minx=min(p[0] for p in pts); maxx=max(p[0] for p in pts)
    miny=min(p[1] for p in pts); maxy=max(p[1] for p in pts)
    sx=(maxx-minx)/CX+1e-9; sy=(maxy-miny)/CY+1e-9
    g=[[' ']*CX for _ in range(CY)]
    for px,py,t in pts:
        c=int((px-minx)/sx); r=int((py-miny)/sy)
        if 0<=c<CX and 0<=r<CY: g[CY-1-r][c]='#'
    print(tag,'x %.0f..%.0f(%.2f) y %.0f..%.0f(%.2f)'%(minx,maxx,sx,miny,maxy,sy))
    for i,row in enumerate(g): print('%3d|'%(CY-1-i)+''.join(row))
# transition in chunks of 60k ticks
for t0 in range(730000,1044001,60000):
    seg=[(a,b,t) for a,b,t in trail if t0<=t<t0+60000]
    if seg: render(seg,tag='T%d-%d'%(t0//1000,(t0+60000)//1000))
