from ctl import Ctl, angdiff
from unstick import spin_to
import time, math, json
b=Ctl(); time.sleep(0.2)
pts=[]
for tgt in range(0,360,15):
    spin_to(b,tgt)
    time.sleep(0.1)
    h=b.heading()
    for k in range(2):
        s=b.scan()
        if not s: continue
        for i,r in enumerate(s):
            if r>0 and r<2.95:
                a=math.radians(h+i*22.5)
                pts.append((round(r*math.cos(a),3),round(r*math.sin(a),3)))
json.dump(pts,open('pano.json','w'))
print(len(pts),'points')
# ascii render
import collections
grid=collections.defaultdict(int)
res=0.1
for x,y in pts:
    grid[(int(round(x/res)),int(round(y/res)))]+=1
xs=[k[0] for k in grid]; ys=[k[1] for k in grid]
for y in range(max(ys),min(ys)-1,-1):
    row=''
    for x in range(min(xs),max(xs)+1):
        c=grid.get((x,y),0)
        row+=('#' if c>2 else ('.' if c>0 else ' '))
    print(f'{y*res:5.1f} {row}')
print('robot at 0,0; +x=east(h0), +y=h90')
