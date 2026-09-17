from ctl import *
import sys
TPM=1960.0
L=open('/bot/src/hill.txt','a')
def log(m): L.write(m+'\n'); L.flush()
def sig(n=6):
    v=[float(rd('d11',0.3)) for _ in range(n)]; v.sort(); return sum(v[1:-1])/(len(v)-2)
def clearance(w):
    h=heading(3); r=ranges()
    i=int(round(((w-h)%360)/22.5))%16
    vals=[r[i], r[(i+1)%16], r[(i-1)%16]]
    vals=[v for v in vals if v>0]
    return min(vals) if vals else 0
def move(w,dist):
    spin_to(w, tol=4)
    return drive(100, ticks=int(dist*TPM), minfront=0.2)
pref=[180,270,90,0]
step=0.2
s0=sig(); log(f"start sig={s0:.3f} d3={rd('d3')}")
fails=0
while fails<3:
    improved=False
    for w in list(pref):
        c=clearance(w)
        if c<step+0.25: log(f"  dir {w} blocked c={c:.2f}"); continue
        res=move(w,step); s1=sig(); st=rd('d3')
        log(f"  moved {w} {res} sig {s0:.3f}->{s1:.3f} {st}")
        if st and ('here=0' not in st or 'goal=0' not in st): log("!!! "+st)
        if s1>s0+0.008:
            s0=s1; improved=True
            pref.remove(w); pref.insert(0,w)
            break
        else:
            move((w+180)%360, step); s0=sig()
            log(f"  back. sig={s0:.3f}")
    if not improved:
        fails+=1; step=max(0.1,step*0.6); log(f"no improvement; step={step}")
log(f"END sig={s0:.3f} d3={rd('d3')}")
stop()
