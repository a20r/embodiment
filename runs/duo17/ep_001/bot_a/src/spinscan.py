import sys,time,math; sys.path.insert(0,'/bot/src'); import ctl as c, rio as m
def spinscan(spd=25, tmax=25):
    pts=[]; h0=c.heading(); acc=0; last=h0; t0=time.time()
    c.drive(spd,-spd)
    while time.time()-t0<tmax:
        h=c.heading(); L=c.lidar()
        if h is None or L is None: continue
        acc+=c.angdiff(h,last); last=h
        for k,r in enumerate(L):
            if r>0: pts.append(((h+22.5*k)%360, r))
        if acc>=350: break
    c.stop(); time.sleep(0.2)
    return pts
def polar(pts, binw=10):
    bins={}
    for a,r in pts: bins.setdefault(int(a//binw)*binw,[]).append(r)
    return {b:(min(v),sorted(v)[len(v)//2]) for b,v in sorted(bins.items())}
def ascii_map(pts, scale=0.1, size=31):
    g=[[' ']*size for _ in range(size)]; cx=cy=size//2
    for a,r in pts:
        x=r*math.sin(math.radians(a)); y=r*math.cos(math.radians(a))  # heading 0 = up, increases clockwise
        i=int(round(cx+x/scale)); j=int(round(cy-y/scale))
        if 0<=i<size and 0<=j<size: g[j][i]='#'
    g[cy][cx]='R'
    return '\n'.join(''.join(row) for row in g)
if __name__=='__main__':
    pts=spinscan()
    print('npts',len(pts),'heading now',c.heading())
    p=polar(pts)
    print(' '.join('%d:%.2f'%(b,v[1]) for b,v in p.items()))
    print(ascii_map(pts, scale=float(sys.argv[1]) if len(sys.argv)>1 else 0.1))
