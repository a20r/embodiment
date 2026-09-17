import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

class Bot:
    def __init__(self):
        self.x=0.0; self.y=0.0
        self.e7=read_float("d7"); self.e8=read_float("d8")
        self.h=read_float("d1")
        self.log=open("/bot/src/tele3.log","a")
        self.t0=time.time()
        self.hist=[]
        self.sm=None
    def update(self, integrate=True):
        e7=read_float("d7"); e8=read_float("d8"); h=read_float("d1")
        d=((e7-self.e7)+(e8-self.e8))/2.0
        self.e7,self.e8=e7,e8
        self.h=h
        if integrate:
            th=math.radians(h)
            self.x+=d*math.sin(th); self.y+=d*math.cos(th)
    def snap(self):
        l=lidar(); d5=read_float("d5"); d6=read_port("d6")
        self.sm = d5 if self.sm is None else 0.7*self.sm+0.3*d5
        rec=dict(t=round(time.time()-self.t0,1),x=round(self.x),y=round(self.y),h=round(self.h,1),
                 d5=d5,sm=round(self.sm,3),d6=d6,l=[round(v,2) for v in l])
        self.log.write(json.dumps(rec)+"\n"); self.log.flush()
        self.hist.append((self.x,self.y,self.sm))
        if len(self.hist)>2000: self.hist=self.hist[-2000:]
        return rec,l
    def best_spot(self):
        if len(self.hist)<20: return None
        return max(self.hist,key=lambda p:p[2])
    def turn_to(self,target_h):
        for _ in range(120):
            h=read_float("d1")
            err=((target_h-h+180)%360)-180
            if abs(err)<9: break
            sp=max(12,min(70,abs(err)*1.6))
            motors(sp if err>0 else -sp, -sp if err>0 else sp)
            time.sleep(0.07)
            self.update()
        motors(0,0)

def clear(l,i):
    v=l[i%16]
    return 3.0 if v<0 else v

def main():
    b=Bot()
    last_ping=0
    prev_front=None; stuck_count=0
    mode="explore"  # or "hold"
    hold_until=0
    while True:
        b.update()
        rec,l=b.snap()
        now=time.time()
        d6=rec["d6"]
        if "here=1" in d6 or "goal=1" in d6:
            print("GOALFLAG:",d6,flush=True)
        if now-last_ping>3:
            write_port("d0", f"POS {b.x/1000:.2f} {b.y/1000:.2f}")
            write_port("d0", "HELLO FROM A. WHERE IS GOAL?")
            last_ping=now
        # hold mode: stay near max-signal spot
        best=b.best_spot()
        if b.sm>2.5 and mode=="explore":
            mode="hold"; hold_until=now+30
            motors(0,0)
            print(f"HOLD at sm={b.sm:.3f} pos=({b.x:.0f},{b.y:.0f})",flush=True)
        if mode=="hold":
            if now>hold_until:
                mode="explore"
                print("resume explore",flush=True)
            else:
                motors(0,0); time.sleep(0.5); continue
        # stuck detection
        front=min(clear(l,0),clear(l,1),clear(l,15))
        if prev_front is not None and abs(front-prev_front)<0.02 and front<0.35:
            stuck_count+=1
        else: stuck_count=0
        prev_front=front
        if stuck_count>=3:
            motors(-80,-80); time.sleep(1.0)
            b.update(integrate=False)
            motors(0,0)
            b.turn_to((b.h+random.choice([90,-90,135,-135]))%360)
            stuck_count=0
            continue
        # steer: if current sm noticeably below max seen, head toward best spot; else keep roaming w/ gradient
        want=None
        if best and best[2]-b.sm>0.06 and math.hypot(best[0]-b.x,best[1]-b.y)>300:
            want=(math.degrees(math.atan2(best[0]-b.x,best[1]-b.y)))%360
        if want is None:
            # gradient run: keep going forward; handled by direction scoring with slight forward pref
            want=b.h
        rel=((want-b.h)%360)/22.5
        bestv=-1e9; besti=0
        for i in range(16):
            c=min(clear(l,i),clear(l,(i+1)%16),clear(l,(i-1)%16))
            angdiff=min((i-rel)%16,(rel-i)%16)
            v=min(c,1.2)*2.0 - angdiff*0.35 + random.uniform(0,0.05)
            if v>bestv: bestv=v; besti=i
        if besti in (0,1,15) and front>0.4:
            steer=0
            if clear(l,1)<0.22 or clear(l,2)<0.15: steer=10
            if clear(l,15)<0.22 or clear(l,14)<0.15: steer=-10
            if besti==1: steer-=6
            if besti==15: steer+=6
            motors(90+steer,90-steer)
            time.sleep(0.22)
        else:
            motors(0,0)
            b.turn_to((b.h+22.5*besti)%360)

if __name__=="__main__":
    main()
