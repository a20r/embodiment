import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

# Beam i points at compass angle (heading + 22.5*i) deg, beam0 = forward.
# Encoders ~mm. heading: compass deg, clockwise positive (left-fwd => increase).

class Bot:
    def __init__(self):
        self.x=0.0; self.y=0.0
        self.e7=read_float("d7"); self.e8=read_float("d8")
        self.h=read_float("d1")
        self.log=open("/bot/src/telemetry.log","a")
        self.t0=time.time()
    def update(self):
        e7=read_float("d7"); e8=read_float("d8"); h=read_float("d1")
        d=((e7-self.e7)+(e8-self.e8))/2.0  # mm
        self.e7,self.e8=e7,e8
        self.h=h
        th=math.radians(h)
        self.x+=d*math.sin(th); self.y+=d*math.cos(th)
    def snap(self):
        l=lidar(); d5=read_float("d5"); d6=read_port("d6"); d2=read_port("d2"); d9=read_port("d9")
        rec=dict(t=round(time.time()-self.t0,1),x=round(self.x),y=round(self.y),h=round(self.h,1),
                 d5=d5,d6=d6,d2=d2,d9=d9,l=[round(v,2) for v in l])
        self.log.write(json.dumps(rec)+"\n"); self.log.flush()
        return rec,l
    def turn_to(self, target):
        # target compass deg
        motors(0,0)
        for _ in range(200):
            h=read_float("d1")
            err=((target-h+180)%360)-180
            if abs(err)<8: break
            sp=max(10,min(60,abs(err)*1.5))
            if err>0: motors(sp,-sp)
            else: motors(-sp,sp)
            time.sleep(0.08)
            self.update()
        motors(0,0)

def main():
    b=Bot()
    last_ping=0
    while True:
        b.update()
        rec,l=b.snap()
        # radio ping
        if time.time()-last_ping>5:
            write_port("d0", json.dumps(dict(who="A",x=round(b.x),y=round(b.y),t=round(time.time()))))
            last_ping=time.time()
        # choose direction: prefer forward if clear
        def clear(i): 
            v=l[i%16]
            return 3.0 if v<0 else v
        front=min(clear(0),clear(1),clear(15))
        if front>0.45:
            # drive forward, slight centering
            b_l=clear(2)+clear(3)
            b_r=clear(13)+clear(14)
            steer=0
            if clear(1)<0.25 or clear(2)<0.15: steer=8    # obstacle left-> turn right
            if clear(15)<0.25 or clear(14)<0.15: steer=-8
            motors(80+steer,80-steer)
            time.sleep(0.25)
        else:
            motors(0,0)
            # pick most open direction, weight forwardness
            best=None;bestv=-1
            for i in range(16):
                v=min(clear(i),clear((i+1)%16),clear((i-1)%16))
                w=v + (0.3 if i in (0,1,15) else 0) + random.uniform(0,0.2)
                if w>bestv: bestv=w; best=i
            target=(b.h+22.5*best)%360
            b.turn_to(target)

if __name__=="__main__":
    main()
