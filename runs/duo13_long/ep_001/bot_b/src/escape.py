import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
def enc():
    a=rd("d6"); b=rd("d9")
    return (float(a) if a is not None else 0.0, float(b) if b is not None else 0.0)
def havg(n=5):
    return sum(heading() for _ in range(n))/n
def backoff(dur=1.2):
    motors(-3,-3); time.sleep(dur); motors(0,0); time.sleep(0.3)
def rotate_to(target, tol=6.0):
    for attempt in range(60):
        h=havg(5)
        dh=(target-h+540)%360-180
        if abs(dh)<tol: return True
        if rd("d0")=="1": backoff()
        rate=3
        t=min(abs(dh)/(2.2*rate), 2.0)
        if dh>0: motors(rate,-rate)
        else: motors(-rate,rate)
        time.sleep(t); motors(0,0); time.sleep(0.25)
    return False
print("start h=%.0f d0=%s" % (havg(), rd("d0")), flush=True)
# target heading ~250 (toward beam4/5 open 0.45-0.7 side) first wiggle out
rotate_to(255, tol=8)
time.sleep(0.3)
l=lidar()
print("after rotate h=%.0f l=%s" % (havg(), l), flush=True)
# drive forward gently, watching bump + front
e0=enc()
motors(2,2)
t0=time.time()
while time.time()-t0<20:
    time.sleep(0.5)
    l=lidar()
    b8=l[8] if l else 0
    b5=l[5] if l else 0
    b11=l[11] if l else 0
    if rd("d0")=="1":
        print("bump at t=%.1f ticks=%d" % (time.time()-t0, int((enc()[0]-e0[0]+enc()[1]-e0[1])/2)), flush=True)
        motors(0,0); break
    if b8<0.25:
        print("wall close b8=%.2f t=%.1f" % (b8, time.time()-t0), flush=True)
        motors(0,0); break
motors(0,0)
print("escaped? h=%.0f l=%s" % (havg(), lidar()), flush=True)
