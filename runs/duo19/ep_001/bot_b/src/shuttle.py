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
MSG = 'GOAL AT NW corner room (here=1 marks it). I am %s. Come toward me: keep d11 rising. Route from your corridor top: N 1.2, W.3,N.3,W.3,N.3,W.4,N.25, W.95, N.55'
def hop_to(wx, wy):
    while True:
        x,y,th = pose.get(); d=math.hypot(wx-x,wy-y)
        if d < 0.08: return
        turn_to_h(pose, math.degrees(math.atan2(wy-y,wx-x)))
        e0=enc(); ts=time.time()
        while time.time()-ts<4:
            r=ranges(); f=[v for v in (r[0],r[1],r[15]) if v>0] if r else []
            if f and min(f)<0.13: stop(); return
            e=enc()
            if e and e0 and ((e[0]-e0[0])+(e[1]-e0[1]))/2/TPU>=min(0.2,d): break
            motors(40,40); time.sleep(0.04)
        stop()
while True:
    way=[pose.get()[:2]]; t0=time.time(); n=0
    while time.time()-t0 < 240 and d11() < 0.5:
        x,y,th = pose.get(); ang=[315,270,0,315][n%4]; n+=1
        reactive_step(pose, x+3*math.cos(math.radians(ang)), y+3*math.sin(math.radians(ang)), steplen=0.3)
        way.append(pose.get()[:2]); log('OUT %d pose=(%.2f,%.2f) d11=%.2f' % (n, *pose.get()[:2], d11()))
        if n%3==0: write_line(8, MSG % 'coming toward you')
    t1=time.time()
    while time.time()-t1 < 1500 and d11() < 0.8:
        write_line(8, MSG % ('waiting for you at goal room entrance, my d11=%.2f' % d11())); time.sleep(6)
    log('WAIT done d11=%.2f after %.0fs' % (d11(), time.time()-t1))
    for wx,wy in reversed(way[:-1]): hop_to(wx,wy)
    for b,dd in ((135,0.5),(90,0.5),(180,0.5),(45,0.5),(225,0.5),(0,0.5),(315,0.5),(270,0.5)):
        if 'here=1' in (read_line(3) or ''): break
        res=drive_until_here(b,dd,speed=35); log('seek',b,res)
        if res[0]=='HERE': break
    tp=time.time()
    while time.time()-tp<150 or d11()>0.6:
        s=read_line(3) or ''; v=d11(); log('PARK %s d11=%.2f' % (s,v))
        write_line(8, MSG % ('PARKED ON GOAL (here=1), my d11=%.2f' % v))
        if 'goal=1' in s: log('GOAL=1 !!!'); sys.exit(0)
        time.sleep(5)
