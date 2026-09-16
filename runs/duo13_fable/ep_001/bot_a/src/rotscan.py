import sys,time,math,json
sys.path.insert(0,'/bot/src')
from ctl import *
def rotscan(secs=36, spd=6):
    pts=[]
    set_speeds(spd,-spd); t0=time.time()
    while time.time()-t0<secs:
        h=heading(); r=ranges()
        if h is None or r is None: continue
        for i,d in enumerate(r):
            if 0<d<2.6:
                a=math.radians(h+i*22.5)
                pts.append((d*math.sin(a), d*math.cos(a)))  # x east, y north
    stop()
    return pts
def ascii_map(pts, res=0.1, extent=2.6):
    n=int(extent/res)
    grid=[[' ']*(2*n+1) for _ in range(2*n+1)]
    for x,y in pts:
        i=int(round(x/res)); j=int(round(y/res))
        if abs(i)<=n and abs(j)<=n: grid[n-j][n+i]='#'
    grid[n][n]='R'
    return '\n'.join(''.join(row) for row in grid)
if __name__=='__main__':
    pts=rotscan(float(sys.argv[1]) if len(sys.argv)>1 else 36)
    json.dump(pts,open('/tmp/pts.json','w'))
    print(ascii_map(pts))
    print("h now", hd(3))
