import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, math, json, random

class Bot:
    def __init__(self):
        self.x=0.0; self.y=0.0
        self.e7=read_float("d7"); self.e8=read_float("d8")
        self.h=read_float("d1")
        self.log=open("/bot/src/tele2.log","a")
        self.t0=time.time()
        self.target=None
        self.hist=[]
        self.stuck=False
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
        rec=dict(t=round(time.time()-self.t0,1),x=round(self.x),y=round(self.y),h=round(self.h,1),
                 d5=d5,d6=d6,l=[round(v,2) for v in l])
        self.log.write(json.dumps(rec)+"\n"); self.log.flush()
        self.hist.append((self.x,self.y,d5))
        if len(self.hist)>500: self.hist=self.hist[-500:]
        return rec,l
    def fit_source(self):
        pts=self.hist[-300:]
        if len(pts)<60: return None
        best=None
        x0,y0=self.x,self.y
        for gx in range(int(x0)-6000,int(x0)+6001,400):
            for gy in range(int(y0)-6000,int(y0)+6001,400):
                num=den=0
                for x,y,s in pts:
                    d=math.hypot(gx-x,gy-y)+1
                    num+=s/d; den+=1/d**2
                k=num/den
                err=sum((s-k/(math.hypot(gx-x,gy-y)+1))**2 for x,y,s in pts)
                if best is None or err<best[0]: best=(err,gx,gy)
        return (best[1],best[2])
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
    last_ping=0; last_fit=0
    prev_front=None; stuck_count=0
    while True:
        b.update()
        rec,l=b.snap()
        now=time.time()
        d6=rec["d6"]
        if "here=1" in d6 or "goal=1" in d6:
            print("GOAL FLAG CHANGED:",d6,flush=True)
            motors(0,0)
            # stay put, keep broadcasting
            write_port("d0", json.dumps(dict(who="A",msg="AT_GOAL",x=round(b.x),y=round(b.y))))
            time.sleep(1)
            continue
        if now-last_ping>4:
            write_port("d0", json.dumps(dict(who="A",x=round(b.x),y=round(b.y),d5=rec["d5"],h=round(b.h))))
            last_ping=now
        if now-last_fit>15:
            t=b.fit_source()
            if t:
                b.target=t
                print(f"fit target={t} pos=({b.x:.0f},{b.y:.0f}) d5={rec['d5']}",flush=True)
            last_fit=now
        # stuck detection: if driving forward but front lidar not changing
        front=min(clear(l,0),clear(l,1),clear(l,15))
        if prev_front is not None and abs(front-prev_front)<0.02 and front<0.35:
            stuck_count+=1
        else:
            stuck_count=0
        prev_front=front
        if stuck_count>=3:
            print("stuck! backing up",flush=True)
            motors(-80,-80); time.sleep(1.0)
            b.update(integrate=False)
            motors(0,0)
            b.turn_to((b.h+random.choice([90,-90,135,-135]))%360)
            stuck_count=0
            continue
        if b.target:
            dx=b.target[0]-b.x; dy=b.target[1]-b.y
            want=(math.degrees(math.atan2(dx,dy)))%360
        else:
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
