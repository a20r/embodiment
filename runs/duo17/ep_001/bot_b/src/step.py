import rb, time, math, sys
from ctl import Bot
def here():
    s=rb.rd('d3',0.3)
    return ('here=1' in s), s
b=Bot()
def drive_until(cond, maxd=0.7, speed=35, timeout=20):
    x0,y0=b.x,b.y; t0=time.time(); h0=b.h; reason='?'
    while time.time()-t0<timeout:
        b.update()
        trav=math.hypot(b.x-x0,b.y-y0)
        if trav>=maxd: reason='maxd'; break
        if b.bump(): reason='bump'; break
        s=b.scan()
        if s and 0<s[0]<0.14: reason='obstacle'; break
        ok,st=here()
        if cond(ok): reason='cond '+st; break
        err=(h0-b.h+180)%360-180
        if s and 0<s[4]<0.6 and 0<s[12]<0.6: err+=max(-12,min(12,(s[4]-s[12])*50))
        corr=max(-15,min(15,err*1.2))
        b.set(speed+corr,speed-corr); time.sleep(0.05)
    b.stop(); return reason, math.hypot(b.x-x0,b.y-y0)
out_h=float(sys.argv[1]) if len(sys.argv)>1 else 0
print('start', here()[1], 'h', b.h, flush=True)
b.turn_to(out_h); print('turned', b.h, flush=True)
r=drive_until(lambda ok: not ok, maxd=0.7); print('OUT:', r, flush=True)
# go a little further out
r2=drive_until(lambda ok: False, maxd=0.12); print('extra:', r2, flush=True)
print('scan', [round(v,2) for v in b.scan()], flush=True)
time.sleep(1.0)
b.turn_to((out_h+180)%360); print('turned back', b.h, flush=True)
r=drive_until(lambda ok: ok, maxd=1.0); print('IN:', r, flush=True)
r2=drive_until(lambda ok: False, maxd=0.08); print('extra:', r2, flush=True)
for i in range(3):
    time.sleep(1); print(rb.rd('d3'), rb.rd('d11'), flush=True)
