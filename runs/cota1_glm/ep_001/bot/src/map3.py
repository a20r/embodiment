import math, pickle
trail,firepos=pickle.load(open('/bot/src/world_trail.pkl','rb'))
# tick-sorted; find rows near fires and after, print hd not stored... recompute hd per row? We stored only x,y,tick.
# Instead: reparse CSV for hd near fire ticks
hd={}
for ln in open('/bot/src/map_input.csv'):
    a=ln.strip().split(',')
    if len(a)!=15: continue
    try: t=int(a[1]); h=float(a[5])
    except: continue
    hd.setdefault(t,[]).append(h)
for fk in (663639,687750):
    hs=[]
    for t in sorted(hd):
        if abs(t-fk)<60: hs+=hd[t]
    print('fire',fk,'compass readings around:',[round(h) for h in hs[:14]])
# direction AFTER each fire: use trail points
def after(tick0,n=40):
    pts=[(x,y) for x,y,t in trail if tick0<=t<=tick0+600]
    return pts
for fk,name in ((687750,'F45'),(663639,'F34')):
    pts=after(fk)
    if len(pts)>10:
        (x0,y0),(x1,y1)=pts[0],pts[-1]
        print(name,'post-fire drift dir: from (%.1f,%.1f) to (%.1f,%.1f)'%(x0,y0,x1,y1))
# render helper
def render(pts,CX=64,CY=56,tag=''):
    minx=min(p[0] for p in pts); maxx=max(p[0] for p in pts)
    miny=min(p[1] for p in pts); maxy=max(p[1] for p in pts)
    sx=(maxx-minx)/CX+1e-9; sy=(maxy-miny)/CY+1e-9
    g=[[' ']*CX for _ in range(CY)]
    for px,py in pts:
        c=int((px-minx)/sx); r=int((py-miny)/sy)
        if 0<=c<CX and 0<=r<CY: g[CY-1-r][c]='#'
    print(tag,'x %.0f..%.0f step%.1f | y %.0f..%.0f'%(minx,maxx,sx,miny,maxy))
    for i,row in enumerate(g): print('%3d|'%(CY-1-i)+''.join(row))
# lap4->5 circuit: ticks 663639..687750
render([(x,y) for x,y,t in trail if 663639<=t<=687750],tag='LAP4-5 CIRCUIT')
# after F45: 687750..730000
render([(x,y) for x,y,t in trail if 687750<=t<=730000],tag='POST-F45 WANDER')
# current subloop: last 12000 ticks
render([(x,y) for x,y,t in trail if t>=1044000],tag='CURRENT SUBLOOP')
