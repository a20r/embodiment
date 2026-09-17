import os, select, math
fd = os.open('/dev/robot/d2', os.O_RDONLY)
r,_,_ = select.select([fd],[],[],3)
data = os.read(fd, 1<<22).decode().strip()
os.close(fd)
pts = [tuple(map(float,p.split(','))) for p in data.split(';') if p]

def grid(pts, ai, aj, W=70, H=30, scale=None):
    cells = [[ ' ' for _ in range(W)] for _ in range(H)]
    a=[p[ai] for p in pts]; b=[p[aj] for p in pts]
    amin,amax=min(a),max(a); bmin,bmax=min(b),max(b)
    if scale: amin,amax,bmin,bmax = scale
    for p in pts:
        i=int((p[ai]-amin)/(amax-amin+1e-9)*(W-1))
        j=int((p[aj]-bmin)/(bmax-bmin+1e-9)*(H-1))
        cells[H-1-j][i]='#'
    return cells,(amin,amax,bmin,bmax)

c,rng = grid(pts,0,1)  # x vs y top view
print("TOP view x(right) x range %.2f..%.2f, y %.2f..%.2f"%rng)
for row in c: print(''.join(row))
c,rng = grid(pts,0,2)  # x vs z side view
print("SIDE view x range %.2f..%.2f, z %.2f..%.2f"%rng)
for row in c: print(''.join(row))
