import sys
pts=[tuple(map(float,l.split(','))) for l in open('/memory/scan0.txt')]
print('n=',len(pts))
# histogram of x in front hemisphere
front=[p for p in pts if p[0]>0.05]
print('front points:', len(front))
if front:
    xs=[p[0] for p in front]; ys=[p[1] for p in front]; zs=[p[2] for p in front]
    print('front x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'%(min(xs),max(xs),min(ys),max(ys),min(zs),max(zs)))
# closest points
spts=sorted(pts)[:20]
print('closest 20:', [(round(a,2),round(b,2),round(c,2)) for a,b,c in spts])
# x>0.1 and |y|<0.2 (dead ahead)
dead=[p for p in pts if p[0]>0.05 and abs(p[1])<0.15]
if dead:
    xs=[p[0] for p in dead]; zs=[p[2] for p in dead]
    print('dead-ahead n=%d x[%.2f,%.2f] z[%.2f,%.2f]'%(len(dead),min(xs),max(xs),min(zs),max(zs)))
