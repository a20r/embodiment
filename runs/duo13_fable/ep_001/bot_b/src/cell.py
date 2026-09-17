from ctl import *
import sys, math, json
TPM=1960.0; CELL=0.5
L=open('/bot/src/cell.txt','a')
def log(m): L.write(m+'\n'); L.flush()
def sig(n=3):
    v=[float(rd('d11',0.3)) for _ in range(n)]; return sum(v)/n
def dist_dir(w,h=None,r=None):
    """range in compass dir w by interpolating beams"""
    if h is None: h=heading(3)
    if r is None: r=ranges()
    i=((w-h)%360)/22.5; i0=int(math.floor(i))%16; i1=(i0+1)%16; f=i-math.floor(i)
    a,b=r[i0],r[i1]
    if a<0 and b<0: return 9
    if a<0: return b
    if b<0: return a
    return min(a,b) if abs(a-b)>0.15 else a*(1-f)+b*f
def walls_here():
    h=heading(3); r=ranges()
    d={w:dist_dir(w,h,r) for w in (0,90,180,270)}
    return d
def face(w): spin_to(w,tol=3,maxs=70)
def step(w, dist=CELL):
    """move dist in compass dir w with side centering; return actual distance (m) and reason"""
    face(w)
    e0=enc(); t0=time.time(); reason='ok'
    while True:
        h=heading(1); r=ranges()
        if h is None or r is None: continue
        e=enc(); trav=((e[0]-e0[0])+(e[1]-e0[1]))/2/TPM
        if trav>=dist: reason='dist'; break
        if time.time()-t0>8: reason='timeout'; break
        front=min([v for v in (r[0],) if v>0] or [9])
        remaining=dist-trav
        if front<0.27: reason='wall'; break
        if rd('d5',0.2)=='1': reason='bump'; break
        err=wrap(w-h)  # heading error
        corr=err*1.2
        # side centering: beam 4 = right(heading+90), beam 12 = left
        rt,lf=r[4],r[12]
        if 0<rt<0.45 and 0<lf<0.45: corr+= (lf-rt)*60   # if left farther, turn left (increase heading?)
        elif 0<rt<0.35: corr+= (0.25-rt)*60
        elif 0<lf<0.35: corr-= (0.25-lf)*60
        corr=max(-25,min(25,corr))
        sp=110 if remaining>0.15 else 70
        motors(sp+corr, sp-corr)
        time.sleep(0.02)
    stop()
    e=enc(); trav=((e[0]-e0[0])+(e[1]-e0[1]))/2/TPM
    # align to front wall if present
    time.sleep(0.1); r=ranges(); f=r[0]
    if 0<f<0.5:
        adj=f-0.25
        if abs(adj)>0.04:
            e0=enc(); sgn=1 if adj>0 else -1
            motors(60*sgn,60*sgn)
            while True:
                e=enc(); t=((e[0]-e0[0])+(e[1]-e0[1]))/2/TPM
                if abs(t)>=abs(adj) or (sgn<0 and rd('d0',0.1)=='1'): break
                time.sleep(0.02)
            stop(); trav+=adj
    return trav, reason
