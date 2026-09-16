from ctl import *
import sys, math
TPM=1960.0
L=open('/bot/src/wall.txt','a')
def log(m): L.write(m+'\n'); L.flush()
def sig(n=4):
    v=[float(rd('d11',0.3)) for _ in range(n)]; return sum(v)/n
def opens():
    """distance to obstacle in 4 compass dirs, using beams within +-22.5"""
    h=heading(3); r=ranges(); out={}
    for w in (0,90,180,270):
        i=((w-h)%360)/22.5; i0=int(math.floor(i))%16; i1=(i0+1)%16
        vals=[v for v in (r[i0],r[i1]) if v>0]
        out[w]=min(vals) if vals else 0
    return out
side=1 if (len(sys.argv)<2 or sys.argv[1]=='R') else -1   # R: wall on heading+90 side
cur=int(sys.argv[2]) if len(sys.argv)>2 else 90
nsteps=int(sys.argv[3]) if len(sys.argv)>3 else 40
STEP=0.25
for k in range(nsteps):
    o=opens(); s=sig(); st=rd('d3')
    log(f"[{k}] cur={cur} sig={s:.3f} open N{o[0]:.2f} E{o[90]:.2f} S{o[180]:.2f} W{o[270]:.2f} {st}")
    if st and ('here=0' not in st or 'goal=0' not in st): log("!!! "+st); break
    right=(cur+90*side)%360; left=(cur-90*side)%360; back=(cur+180)%360
    if o[right]>0.6: nd=right
    elif o[cur]>0.45: nd=cur
    elif o[left]>0.6: nd=left
    else: nd=back
    dist=STEP if nd==cur else 0.45
    cur=nd
    spin_to(nd,tol=4)
    res=drive(120, ticks=int(dist*TPM), minfront=0.22)
    log(f"    -> {nd} {res}")
stop(); log("END")
