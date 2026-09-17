import math, pickle
rows=[]
for ln in open('/bot/src/map_input.csv'):
    a=ln.strip().split(',')
    if len(a)!=15: continue
    try: rows.append((int(a[1]), float(a[5]), float(a[6])))
    except: pass
rows.sort()
fires={472527:'F23',663669:'F34',687778:'F45'}
x=y=0.0; prev=None; trail=[]; firepos={}; hd_prev=None
nseg=0; totlen=0.0
for tick,hd,sp in rows:
    if abs(sp)>2.5: sp=math.copysign(2.5,sp)
    if hd_prev is not None and abs((hd-hd_prev+180)%360-180)>60: hd=hd_prev
    hd_prev=hd
    for fk in list(fires):
        if abs(tick-fk)<=40 and fk not in firepos:
            firepos[fk]=(x,y); print('FIRE',fires[fk],'tick',tick,'at (%.1f,%.1f)'%(x,y))
    if prev is None: prev=tick; trail.append((x,y,tick)); continue
    dt=(tick-prev)/100.0; prev=tick
    if dt<=0: continue
    if dt>2.0: nseg+=1; trail.append((x,y,tick)); continue
    r=math.radians(hd); step=sp*dt
    x+=step*math.cos(r); y+=step*math.sin(r); totlen+=step
    trail.append((x,y,tick))
print('rows',len(rows),'segments',nseg+1,'len %.1f'%totlen,'end (%.1f,%.1f)'%(x,y))
pickle.dump((trail,firepos), open('/bot/src/world_trail.pkl','wb'))
# ASCII render: 4m cells
minx=min(p[0] for p in trail); maxx=max(p[0] for p in trail)
miny=min(p[1] for p in trail); maxy=max(p[1] for p in trail)
CX,CY=60,110
sx=(maxx-minx)/CX+1e-9; sy=(maxy-miny)/CY+1e-9
grid=[[' ']*CX for _ in range(CY)]
for px,py,tick in trail:
    c=int((px-minx)/sx); r=int((py-miny)/sy)
    if 0<=c<CX and 0<=r<CY: grid[CY-1-r][c]='#'
for fk,(px,py) in firepos.items():
    c=int((px-minx)/sx); r=int((py-miny)/sy)
    if 0<=c<CX and 0<=r<CY: grid[CY-1-r][c]=fires[fk][-1]
c=int((x-minx)/sx); r=int((y-miny)/sy)
if 0<=c<CX and 0<=r<CY: grid[CY-1-r][c]='E'
print('x: %.0f..%.0f (col*%.1f+%.0f)  y: %.0f..%.0f (row flipped)'%(minx,maxx,sx,minx,miny,maxy))
for r,row in enumerate(grid): print('%3d|'%(CY-1-r)+''.join(row))
