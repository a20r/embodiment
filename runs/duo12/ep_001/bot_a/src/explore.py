import time, math, json, sys
from robot import Bot, angdiff

SCALE=0.0006  # units per encoder count (approx)

class Nav:
    def __init__(self,bot):
        self.bot=bot; self.x=0.0; self.y=0.0
        try:
            import json as _j
            p=_j.load(open("/memory/pose.json"))
            self.x=p["x"]; self.y=p["y"]
        except Exception: pass
        self.laste=bot.enc()
        self._lastsave=0
    def update(self):
        e=self.bot.enc()
        d=((e[0]-self.laste[0])+(e[1]-self.laste[1]))/2*SCALE
        self.laste=e
        h=self.bot.heading()
        if h is None: return
        rad=math.radians(h)
        self.x+=d*math.cos(rad); self.y+=d*math.sin(rad)
        if time.time()-self._lastsave>2:
            self._lastsave=time.time()
            try:
                import json as _j
                _j.dump({"x":self.x,"y":self.y},open("/memory/pose.json","w"))
            except Exception: pass

def turn_to(bot,nav,target,tol=7,timeout=20):
    end=time.time()+timeout
    while time.time()<end:
        h=bot.heading(); d=angdiff(target,h)
        if abs(d)<tol:
            bot.stop(); return True
        s=30 if abs(d)>30 else 8
        if d>0: bot.wheels(-s,s)
        else: bot.wheels(s,-s)
        time.sleep(0.12)
        nav.update()
    bot.stop(); return False

def drive(bot,nav,dist_units,hold_heading,speed=100,timeout=40):
    start=(nav.x,nav.y); end=time.time()+timeout
    while time.time()<end:
        nav.update()
        if math.hypot(nav.x-start[0],nav.y-start[1])>=dist_units: break
        r=bot.ranges()
        if r:
            cand=[x for x in (r[0],r[1],r[15]) if x>=0]
            front=min(cand) if cand else 9
            side=[x for x in (r[2],r[14]) if x>=0]
            if front<0.30 or (side and min(side)<0.15): break
        h=bot.heading()
        corr=angdiff(hold_heading,h)*1.5
        corr=max(-20,min(20,corr))
        bot.wheels(int(speed-corr),int(speed+corr))
        time.sleep(0.12)
    bot.stop()
    nav.update()

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/explore.log","a",buffering=1)
    lastdir=None
    t0=time.time()
    seq=0
    hist=[]
    lastlog=0
    while time.time()-t0<3300:
        time.sleep(0.25)
        nav.update()
        r=bot.ranges(); h=bot.heading()
        st=bot.status()
        d11=bot.d11.last(3)
        rx=[l for l in bot.rx.take() if l.strip()]
        if time.time()-lastlog>1.0 or rx or st.get("here") or st.get("goal"):
            lastlog=time.time()
            log.write(json.dumps({"t":round(time.time()-t0,1),"x":round(nav.x,3),"y":round(nav.y,3),
                "h":round(h,1),"r":[round(v,2) for v in r],"st":st,"d11":d11,"rx":rx})+"\n")
        bot.tx(f"PING x={nav.x:.2f} y={nav.y:.2f} seq={seq}")
        seq+=1
        hist.append((nav.x,nav.y))
        if len(hist)>8: hist.pop(0)
        if len(hist)==8 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.05:
            # stuck: back up and turn random
            import random
            bot.wheels(-80,-80); 
            end=time.time()+2.0
            while time.time()<end: time.sleep(0.1); nav.update()
            bot.stop()
            turn_to(bot,nav,(bot.heading()+random.choice([90,-90,180]))%360)
            hist=[]
            continue
        if st.get("here")==1 or st.get("goal")==1:
            log.write("FLAG CHANGED, stopping to think\n")
            bot.stop(); time.sleep(5); continue
        # choose direction: beams sorted by range, prefer small turn, avoid reversing
        best=None;bestscore=-9
        for i in range(16):
            beam_abs=(h+22.5*i)%360
            if r[i]<0.45: continue
            score=min(r[i],2.0)
            score-=abs(angdiff(beam_abs,h))/180*0.5
            if lastdir is not None:
                back=angdiff(beam_abs,(lastdir+180)%360)
                if abs(back)<40: score-=1.5
            if score>bestscore: bestscore=score;best=i
        if best is None:
            # boxed in: turn around
            target=(h+180)%360
            turn_to(bot,nav,target)
            lastdir=target
            continue
        target=(h+22.5*best)%360
        turn_to(bot,nav,target)
        lastdir=target
        dist=max(0.2,min(r[best]-0.35,1.2))
        drive(bot,nav,dist,target)
    bot.stop()

if __name__=="__main__":
    main()
