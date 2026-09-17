import time, math, json
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

CELL=0.4
def cell(x,y): return (round(x/CELL),round(y/CELL))

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/explore2.log","a",buffering=1)
    visited={}
    try:
        for k,v in json.load(open("/memory/visited.json")).items():
            a,b=k.split(","); visited[(int(a),int(b))]=v
    except Exception: pass
    seq=0; t0=time.time(); lastlog=0; lastsave=0
    hist=[]
    while time.time()-t0<3000:
        time.sleep(0.2)
        nav.update()
        c=cell(nav.x,nav.y); visited[c]=visited.get(c,0)+1
        r=bot.ranges(); h=bot.heading(); st=bot.status()
        v=[float(x) for x in bot.d11.last(3)]; v=sum(v)/len(v) if v else -1
        rx=[l for l in bot.rx.take() if l.strip()]
        if time.time()-lastlog>1.0 or rx:
            lastlog=time.time()
            log.write(json.dumps({"t":round(time.time()-t0,1),"x":round(nav.x,3),"y":round(nav.y,3),
              "h":round(h,1),"r":[round(q,2) for q in r],"st":st,"v":round(v,3),"rx":rx,"d0":bot.d0.latest,"d5":bot.d5.latest})+"\n")
        if time.time()-lastsave>10:
            lastsave=time.time()
            json.dump({f"{a},{b}":n for (a,b),n in visited.items()},open("/memory/visited.json","w"))
        bot.tx(f"botB PING x={nav.x:.2f} y={nav.y:.2f} seq={seq}"); seq+=1
        if st.get("here")==1 or st.get("goal")==1:
            log.write(f"FLAG! st={st} at {nav.x:.2f},{nav.y:.2f}\n"); bot.stop()
            bot.tx("botB here=1 I AM AT GOAL, STAYING PUT")
            time.sleep(2); continue
        hist.append((nav.x,nav.y))
        if len(hist)>10: hist.pop(0)
        if len(hist)==10 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.05:
            import random
            bot.wheels(-80,-80); end=time.time()+1.5
            while time.time()<end: time.sleep(0.1); nav.update()
            bot.stop(); turn_to(bot,nav,(bot.heading()+random.choice([90,-90,180]))%360)
            hist=[]; continue
        best=None;bestscore=-1e9
        for i in range(16):
            if r[i]<0.5: continue
            beam=(h+22.5*i)%360; rad=math.radians(beam)
            d=min(r[i]-0.3,1.0)
            tx_=nav.x+d*math.cos(rad); ty=nav.y+d*math.sin(rad)
            nov=visited.get(cell(tx_,ty),0)+0.5*visited.get(cell(nav.x+0.5*d*math.cos(rad),nav.y+0.5*d*math.sin(rad)),0)
            import math as _m
            dist_t=_m.hypot(tx_-(-2.63),ty-6.61)
            import math as _m2
            cur_d=_m2.hypot(nav.x-(-2.63),nav.y-6.61)
            score=-nov*0.35 + min(r[i],1.5)*0.5 - abs(angdiff(beam,h))/180*0.4 + (cur_d-dist_t)*3.0
            if score>bestscore: bestscore=score;best=i
        if best is None:
            turn_to(bot,nav,(h+180)%360); drive(bot,nav,0.3,(h+180)%360,speed=60); continue
        beam=(h+22.5*best)%360
        turn_to(bot,nav,beam)
        drive(bot,nav,max(0.2,min(r[best]-0.3,1.0)),beam,speed=100)
    bot.stop()

if __name__=="__main__":
    main()
