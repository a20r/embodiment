import sys, time, math; sys.path.insert(0, '/bot/src')
import explore as E
from explore import *
from seek import drive_until_here
def d11():
    vs=[]
    for _ in range(3):
        try: vs.append(float(read_line(11,0.3)))
        except: pass
    return sum(vs)/len(vs) if vs else 0
E.POSE = pose = Pose(0.0, 0.0)
way = [pose.get()[:2]]
MSG = 'GOAL AT NW corner room (here=1 marks it). I am %s. Come toward me: keep d11 rising. Route from your corridor top: N 1.2, W.3,N.3,W.3,N.3,W.4,N.25, W.95, N.55'
# phase 1: go SE-ish toward A until in radio range (d11>=0.55), recording waypoints
t0=time.time(); n=0
while time.time()-t0 < 300 and d11() < 0.55:
    x,y,th = pose.get(); ang=[315,270,0,315][n%4]; n+=1
    reactive_step(pose, x+3*math.cos(math.radians(ang)), y+3*math.sin(math.radians(ang)), steplen=0.3)
    way.append(pose.get()[:2]); log('OUT %d pose=(%.2f,%.2f) d11=%.2f' % (n, *pose.get()[:2], d11()))
    if n%3==0: write_line(8, MSG % 'coming toward you')
# phase 2: wait for A to approach
t1=time.time()
while time.time()-t1 < 420 and d11() < 0.85:
    write_line(8, MSG % ('waiting for you, my d11=%.2f' % d11())); time.sleep(6)
log('WAIT done d11=%.2f after %.0fs' % (d11(), time.time()-t1))
# phase 3: replay waypoints back in small hops, pausing for A
for wx,wy in reversed(way[:-1]):
    while True:
        x,y,th = pose.get(); d=math.hypot(wx-x,wy-y)
        if d < 0.08: break
        b=math.degrees(math.atan2(wy-y,wx-x)); turn_to_h(pose,b)
        e0=enc(); hop=min(0.15,d); ts=time.time()
        while time.time()-ts<4:
            r=ranges(); f=[v for v in (r[0],r[1],r[15]) if v>0] if r else []
            if f and min(f)<0.13: break
            e=enc()
            if e and e0 and ((e[0]-e0[0])+(e[1]-e0[1]))/2/TPU>=hop: break
            motors(40,40); time.sleep(0.04)
        stop()
        w0=time.time()
        while d11()<0.85 and time.time()-w0<45: time.sleep(1)
        log('BACK hop pose=(%.2f,%.2f) d11=%.2f waited %.0f' % (*pose.get()[:2], d11(), time.time()-w0))
        write_line(8, MSG % 'leading you, follow me (keep d11>0.85)')
        if time.time()-w0>=45 and d11()<0.6: break
# phase 4: seek goal and park
for b,dd in ((135,0.5),(90,0.5),(180,0.5),(45,0.5),(225,0.5),(0,0.5),(315,0.5),(270,0.5)):
    if 'here=1' in (read_line(3) or ''): break
    res=drive_until_here(b,dd,speed=35); log('seek',b,res)
    if res[0]=='HERE': break
while True:
    s=read_line(3) or ''; v=d11(); log('PARK %s d11=%.2f' % (s,v))
    write_line(8, MSG % ('PARKED ON GOAL (here=1), my d11=%.2f' % v))
    if 'goal=1' in s: log('GOAL=1 !!!'); break
    time.sleep(5)
