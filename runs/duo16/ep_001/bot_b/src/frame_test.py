import json, math
S = json.load(open('/memory/scans.json'))
def grid(pts, res=0.1):
    g=set()
    for x,y,z in pts:
        if -0.05<z<0.15:
            g.add((round(x/res), round(y/res)))
    return g
def overlap(g1,g2):
    if not g1 or not g2: return 0
    return len(g1&g2)/min(len(g1),len(g2))
def rot(pts, deg):
    a=math.radians(deg); c,s=math.cos(a),math.sin(a)
    return [(x*c-y*s, x*s+y*c, z) for x,y,z in pts]
A,B,C = S['A'],S['B'],S['C']
gA,gB,gC = grid(A),grid(B),grid(C)
print('sizes', len(gA), len(gB), len(gC))
print('A-B   raw      :', round(overlap(gA,gB),3))
print('A-rotB(+79.3)   :', round(overlap(gA,grid(rot(B,79.3))),3))
print('A-C   raw      :', round(overlap(gA,gC),3))
print('A-rotC(-100.4)  :', round(overlap(gA,grid(rot(C,-100.4))),3))
print('B-rotC(-21.1)   :', round(overlap(gB,grid(rot(C,-21.1))),3))
print('B-C   raw      :', round(overlap(gB,gC),3))
# also try rotation about centroid (in case world frame + robot moved)
def rot_about(pts, deg, cx, cy):
    a=math.radians(deg); c,s=math.cos(a),math.sin(a)
    return [((x-cx)*c-(y-cy)*s+cx, (x-cx)*s+(y-cy)*c+cy, z) for x,y,z in pts]
def cent(pts):
    return (sum(p[0] for p in pts)/len(pts), sum(p[1] for p in pts)/len(pts))
cA=cent(A); cC=cent(C)
print('centroids', cA, cC)
print('A-rotC about centroid(-100.4):', round(overlap(gA, grid(rot_about(C,-100.4,cA[0],cA[1]))),3))
