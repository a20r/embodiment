import sys, time, math
sys.path.insert(0,'/bot/src')
from rob import *
def lastline(name, timeout=0.3):
    d = read_port(name, timeout).decode(errors='replace')
    lines=[l for l in d.strip().split('\n') if l.strip()]
    return lines[-1] if lines else ''
def cloud_summary(pts, tag):
    if not pts: print(tag,'empty'); return
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]
    cx=sum(xs)/len(xs); cy=sum(ys)/len(ys)
    print('%s n=%d x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] c=(%.3f,%.3f)'%(tag,len(pts),min(xs),max(xs),min(ys),max(ys),min(zs),max(zs),cx,cy))
    import pickle
A=scan(1.5); cloud_summary(A,'A  ')
def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d
h0=float(lastline('d4'))
write_port('d1','6'); write_port('d7','-6')
t0=time.time()
target=179.0
while time.time()-t0<25:
    time.sleep(0.7)
    h=float(lastline('d4'))
    if abs(angdiff(h,h0))>target: break
write_port('d1','0'); write_port('d7','0')
time.sleep(0.6)
h1=float(lastline('d4'))
print('spun from %.1f to %.1f (%.1f deg in %.1fs)'%(h0,h1,angdiff(h1,h0),time.time()-t0))
B=scan(1.5); cloud_summary(B,'B  ')
# spin back same amount
write_port('d1','-6'); write_port('d7','6')
t0=time.time()
while time.time()-t0<25:
    time.sleep(0.7)
    h=float(lastline('d4'))
    if abs(angdiff(h,h1))>target: break
write_port('d1','0'); write_port('d7','0')
time.sleep(0.6)
h2=float(lastline('d4'))
print('spun back to %.1f'%h2)
C=scan(1.5); cloud_summary(C,'C  ')
print('d0=',lastline('d0'),'d5=',lastline('d5'),'d11=',lastline('d11'))
import json
json.dump({'A':A,'B':B,'C':C}, open('/memory/scans.json','w'))
