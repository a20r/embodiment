import time, math
from robot import Robot
r=Robot(); r.stop(); time.sleep(0.5)
TICKS=544.0
le,re=r.enc()
x=y=0.0
def upd():
    global x,y,le,re
    l2,r2=r.enc(); h2=r.heading()
    if l2 is None or h2 is None: return
    dl=(l2-le)/TICKS; dr=(r2-re)/TICKS; le,re=l2,r2
    d=(dl+dr)/2; rad=math.radians(h2)
    x+=d*math.sin(rad); y+=d*math.cos(rad)
    return d
# 1) straight 1.0m
l0,r0=r.enc()
t0=time.time()
while time.time()-t0<8:
    l1,r1=r.enc()
    if l1 is None: continue
    if ((l1-l0)+(r1-r0))/2>=1.0*TICKS: break
    r.wheels(20,20); time.sleep(0.05)
r.stop(); time.sleep(0.5)
d=upd() or 0
print('STRAIGHT cmd 1.0m -> odom d=%.2f m, h=%.0f->%.0f' % (d, h0 if False else r.heading(), r.heading()))
# 2) pivot 180
h1=r.heading(); tg=(h1+180)%360; t0=time.time()
l0,r0=r.enc()
while time.time()-t0<8:
    hc=r.heading(); e=(tg-hc+180)%360-180
    if abs(e)<=5: break
    v=max(12,min(38,abs(e)*1.4))
    if e>0: r.wheels(v,-v)
    else: r.wheels(-v,v)
    time.sleep(0.04)
r.stop(); time.sleep(0.5)
d=upd() or 0
print('PIVOT 180 -> odom drift d=%.2f m (should be ~0), final h=%.0f' % (d, r.heading()))
# 3) 8x pivot 90
for i in range(8):
    h1=r.heading(); tg=(h1+90)%360; t0=time.time()
    while time.time()-t0<6:
        hc=r.heading(); e=(tg-hc+180)%360-180
        if abs(e)<=5: break
        v=max(12,min(38,abs(e)*1.4))
        if e>0: r.wheels(v,-v)
        else: r.wheels(-v,v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.2)
    upd()
print('8x90PIVOT -> odom drift (%.2f,%.2f) h=%.0f (want ~0,0,~h1+720)' % (x,y,r.heading()))
r.stop()
