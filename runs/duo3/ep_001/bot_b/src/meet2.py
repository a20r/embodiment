import time, statistics, re, collections
from agent import Agent
from grid3 import DIRS
a=Agent(); b=a.b
h=b.heading(); a.facing=round(h/90)%4*90
a.lg('MEET2 start')

def sig(dur=2.0):
    vals=[]; t0=time.time()
    while time.time()-t0<dur:
        b.p[6].poll()
        vals+=[float(x) for x in b.p[6].queue]; b.p[6].queue.clear()
        b.p[0].poll()
        time.sleep(0.05)
    return statistics.mean(vals) if vals else 0.0

def see():
    b.p[0].poll(); return b.p[0].last=='1'

t0=time.time()
while time.time()-t0<300:
    base=sig()
    a.lg(f'M2 base={base:.3f} pos={(a.cx,a.cy)} see={see()}')
    if base>1.1: break
    if see():
        # drive toward longest-changing beam? simple: find beam whose value < expected wall and blobby: pick min beam? robot appears as short reading in open dir
        s=b.sc()
        # candidate: beams with 0.2<r<2.0; choose the one that fluctuates: take two scans
        s2=b.sc()
        cand=[(abs(s[i]-s2[i]),i) for i in range(16)]
        a.lg(f'M2 SEE scans {[round(x,2) for x in s]}')
    best=(base-0.02,None)
    for d in (0,90,180,270):
        # is it open?
        b.spin_to(d); a.facing=d
        s=b.sc()
        if s[0]<0.5: continue
        if a.go(d):
            a.cx+=DIRS[d][0]; a.cy+=DIRS[d][1]
            v=sig()
            a.lg(f'M2 try {d} -> {v:.3f}')
            if v>best[0]: best=(v,d)
            back=(d+180)%360
            if a.go(back):
                a.cx+=DIRS[back][0]; a.cy+=DIRS[back][1]; a.facing=back
            else:
                a.lg('M2 cant return, adopt current')
                break
    if best[1] is None:
        a.lg('M2 no better dir; hold 5s'); time.sleep(5); continue
    d=best[1]
    if a.go(d):
        a.cx+=DIRS[d][0]; a.cy+=DIRS[d][1]; a.facing=d
        a.lg(f'M2 commit {d} sig now {sig():.3f}')
b.stop()
fs=sig()
a.lg(f'M2 done sig={fs:.3f}')
for i in range(3):
    b.radio_send(f'A: near you (sig={fs:.2f}). Now I explore slowly, FOLLOW ME. On goal: park+GOALFOUND.')
    time.sleep(1)
a.run()
