import sys, time
sys.path.insert(0,'/bot/src')
from rob import *

def lastline(name, timeout=0.3):
    d = read_port(name, timeout).decode(errors='replace')
    lines=[l for l in d.strip().split('\n') if l.strip()]
    return lines[-1] if lines else ''

def obj_stats(pts, xr, yr, zr):
    sel=[p for p in pts if xr[0]<=p[0]<=xr[1] and yr[0]<=p[1]<=yr[1] and zr[0]<=p[2]<=zr[1]]
    if not sel: return None
    n=len(sel)
    cx=sum(p[0] for p in sel)/n; cy=sum(p[1] for p in sel)/n; cz=sum(p[2] for p in sel)/n
    return (n, round(cx,3), round(cy,3), round(min(p[0] for p in sel),3), round(max(p[0] for p in sel),3))

def drive(l, r, secs):
    write_port('d1', str(l)); write_port('d7', str(r))
    time.sleep(secs)
    write_port('d1','0'); write_port('d7','0')

A = scan(1.2)
print('A n=%d' % len(A))
print(' front obj:', obj_stats(A,(0.05,0.6),(-0.3,0.3),(-0.2,0.2)))
print(' back obj:', obj_stats(A,(-0.6,-0.15),(-0.3,0.3),(-0.2,0.2)))
o9=float(lastline('d9')); o6=float(lastline('d6'))
drive(10,10,2.0)
time.sleep(0.4)
o9b=float(lastline('d9')); o6b=float(lastline('d6'))
print('odom delta: d9=%.0f d6=%.0f' % (o9b-o9, o6b-o6))
B = scan(1.2)
print('B n=%d' % len(B))
print(' front obj:', obj_stats(B,(0.05,0.6),(-0.3,0.3),(-0.2,0.2)))
print(' back obj:', obj_stats(B,(-0.6,-0.15),(-0.3,0.3),(-0.2,0.2)))
print(' d4:', lastline('d4'), 'd11:', lastline('d11'), 'd0:', lastline('d0'))
# try to push further
drive(10,10,3.0)
time.sleep(0.4)
C = scan(1.2)
print('C front obj:', obj_stats(C,(0.05,0.6),(-0.3,0.3),(-0.2,0.2)))
print('C back obj:', obj_stats(C,(-0.6,-0.15),(-0.3,0.3),(-0.2,0.2)))
print(' d0:', lastline('d0'), 'd5:', lastline('d5'), 'd11:', lastline('d11'), 'status:', status())
