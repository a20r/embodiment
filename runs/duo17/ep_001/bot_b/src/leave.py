import rb, time, math, sys
from ctl import Bot
def here():
    s=rb.rd('d3',0.3); return ('here=1' in s), s
b=Bot()
def drive_until(cond, maxd=0.7, speed=35, timeout=20, minfront=0.14):
    x0,y0=b.x,b.y; t0=time.time(); h0=b.h; reason='?'
    while time.time()-t0<timeout:
        b.update()
        trav=math.hypot(b.x-x0,b.y-y0)
        if trav>=maxd: reason='maxd'; break
        if b.bump(): reason='bump'; break
        s=b.scan()
        if s and 0<s[0]<minfront: reason='obstacle'; break
        ok,st=here()
        if cond(ok): reason='cond '+st; break
        err=(h0-b.h+180)%360-180
        if s and 0<s[4]<0.6 and 0<s[12]<0.6: err+=max(-12,min(12,(s[4]-s[12])*50))
        corr=max(-15,min(15,err*1.2))
        b.set(speed+corr,speed-corr); time.sleep(0.05)
    b.stop(); return reason, math.hypot(b.x-x0,b.y-y0)
def free_at(s, hdg):
    # distance along absolute heading hdg using nearest beam
    k=int(round(((hdg-b.h)%360)/22.5))%16
    v=[s[k], s[(k-1)%16], s[(k+1)%16]]
    v=[x for x in v if x>0]
    return min(v) if v else 0.3
first=float(sys.argv[1]) if len(sys.argv)>1 else 90
legs=[]; last=None
print('start', here()[1], flush=True)
for i in range(6):
    s=b.scan()
    cands=[]
    for hd in (0,90,180,270):
        if last is not None and (hd-last)%360==180: continue
        f=free_at(s,hd); cands.append((f,hd))
    if i==0: cands=[(free_at(s,first),first)]+[c for c in cands if c[1]!=first]
    else: cands.sort(reverse=True)
    f,hd=cands[0]
    if f<0.3: print('no way, free', cands, flush=True); break
    b.turn_to(hd)
    r,d=drive_until(lambda ok: not ok, maxd=max(0.05,f-0.16))
    legs.append((hd,d)); last=hd
    print('leg', hd, round(d,2), r, flush=True)
    if r.startswith('cond'):
        r2=drive_until(lambda ok: False, maxd=0.1); legs[-1]=(hd,d+r2[1]); break
print('OUT legs', [(h,round(d,2)) for h,d in legs], here()[1], flush=True)
time.sleep(2.0)
for hd,d in reversed(legs):
    b.turn_to((hd+180)%360)
    r=drive_until(lambda ok: False, maxd=d, timeout=20)
    print('back', (hd+180)%360, round(r[1],2), r[0], flush=True)
ok,st=here()
if not ok:
    r=drive_until(lambda ok: ok, maxd=0.5); print('seek', r, flush=True)
    r=drive_until(lambda ok: False, maxd=0.08)
for i in range(3):
    time.sleep(1); print(rb.rd('d3'), rb.rd('d11'), flush=True)
