import time, math, statistics, random
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

WPS=[(-1.0,3.8),(-0.6,4.6),(-1.2,5.2),(-2.0,5.9),(-2.35,6.1)]
def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/wp.log","a",buffering=1)
    hist=[]
    for wi,(wx,wy) in enumerate(WPS):
        t0=time.time()
        while time.time()-t0<90:
            nav.update()
            st=bot.status()
            if st.get("here"):
                bot.stop(); log.write(f"HERE=1 pos=({nav.x:.2f},{nav.y:.2f})\n")
                bot.tx("botB here=1"); time.sleep(1); continue
            d=math.hypot(wx-nav.x,wy-nav.y)
            if d<0.3: break
            hist.append((nav.x,nav.y))
            if len(hist)>10: hist.pop(0)
            if len(hist)==10 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.05:
                bot.wheels(-85,-85); e=time.time()+1.2
                while time.time()<e: time.sleep(0.1); nav.update()
                bot.stop(); hist=[]
            bearing=math.degrees(math.atan2(wy-nav.y,wx-nav.x))%360
            r=bot.ranges(); h=bot.heading()
            best=None;bs=-1e9
            for i in range(16):
                if r[i]<0.4: continue
                beam=(h+22.5*i)%360
                sc=-abs(angdiff(beam,bearing))/45+min(r[i],1.2)
                if sc>bs: bs=sc;best=i
            if best is None:
                bot.wheels(-70,-70); time.sleep(1); bot.stop(); continue
            tgt=(h+22.5*best)%360
            if abs(angdiff(tgt,h))>25: turn_to(bot,nav,tgt); continue
            corr=max(-20,min(20,angdiff(tgt,h)*1.3))
            bot.wheels(int(80-corr),int(80+corr)); time.sleep(0.15)
        vs=[float(x) for x in bot.d11.last(3)]; vv=statistics.median(vs) if vs else -1
        log.write(f"wp{wi} reached-ish pos=({nav.x:.2f},{nav.y:.2f}) v={vv:.3f}\n")
    bot.stop(); log.write("wps done\n")

if __name__=="__main__":
    main()
