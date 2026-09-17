import math, pickle
trail=pickle.load(open('/bot/src/world_trail.pkl','rb'))
def bbox(pts): 
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    return 'x %.0f..%.0f y %.0f..%.0f n=%d'%(min(xs),max(xs),min(ys),max(ys),len(pts))
for t0 in range(730000,1164000,60000):
    seg=[(a,b) for a,b,t in trail if t0<=t<t0+60000]
    print('T%d-%d:'%(t0//1000,(t0+60000)//1000), bbox(seg) if seg else 'empty')
def render(pts,CX=56,CY=44,tag='',marks=()):
    minx=min(p[0] for p in pts); maxx=max(p[0] for p in pts)
    miny=min(p[1] for p in pts); maxy=max(p[1] for p in pts)
    sx=(maxx-minx)/CX+1e-9; sy=(maxy-miny)/CY+1e-9
    g=[[' ']*CX for _ in range(CY)]
    for px,py in pts:
        c=int((px-minx)/sx); r=int((py-miny)/sy)
        if 0<=c<CX and 0<=r<CY: g[CY-1-r][c]='#'
    for (mx,my,ch) in marks:
        c=int((mx-minx)/sx); r=int((my-miny)/sy)
        if 0<=c<CX and 0<=r<CY: g[CY-1-r][c]=ch
    print(tag,'x %.0f..%.0f y %.0f..%.0f'%(minx,maxx,miny,maxy))
    for i,row in enumerate(g): print('%3d|'%(CY-1-i)+''.join(row))
# where does it leave the circuit region? render 790k-970k
render([(a,b) for a,b,t in trail if 790000<=t<910000],tag='T790-910',marks=((7.75,-107.2),'Z'))
render([(a,b) for a,b,t in trail if 910000<=t<1030000],tag='T910-1030',marks=((7.75,-107.2),'Z'))
render([(a,b) for a,b,t in trail if 1030000<=t<=1162393],tag='T1030-NOW',marks=((7.75,-107.2),'Z'))
