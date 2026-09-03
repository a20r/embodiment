import time, math, statistics, random
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

def vmeas(bot,dur=1.0):
    bot.d11.take(); time.sleep(dur)
    vs=[float(x) for x in bot.d11.take() if x]
    return statistics.median(vs) if vs else -1

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/bug.log","a",buffering=1)
    t0=time.time(); seq=0; mode="climb"; heading=bot.heading()
    v=vmeas(bot); hist=[]
    lastlog=0
    while time.time()-t0<2500:
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        if st.get("here"):
            bot.stop(); log.write(f"HERE=1!!! pos=({nav.x:.2f},{nav.y:.2f})\n")
            bot.tx("botB here=1 AT GOAL NOW. botA be here too!")
            time.sleep(1.5); continue
        nav.update()
        hist.append((nav.x,nav.y))
        if len(hist)>12: hist.pop(0)
        if len(hist)==12 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.05:
            bot.wheels(-85,-85); e=time.time()+1.2
            while time.time()<e: time.sleep(0.1); nav.update()
            bot.stop(); hist=[]
            heading=(bot.heading()+random.choice([90,-90,150]))%360
            log.write("unstick\n")
        if mode=="climb":
            r=bot.ranges(); h=bot.heading()
            best=None;bs=-1e9
            for i in range(16):
                if r[i]<0.45: continue
                beam=(h+22.5*i)%360
                sc=-abs(angdiff(beam,heading))/60+min(r[i],1.0)
                if sc>bs: bs=sc;best=i
            if best is None:
                heading=(heading+180)%360; continue
            tgt=(h+22.5*best)%360
            turn_to(bot,nav,tgt)
            drive(bot,nav,0.7,tgt,speed=100)
            nv=vmeas(bot,0.9)
            bot.tx(f"botB PING x={nav.x:.2f} y={nav.y:.2f} d11={nv:.3f} seq={seq}"); seq+=1
            log.write(f"climb t={time.time()-t0:4.0f} v {v:.3f}->{nv:.3f} pos=({nav.x:.2f},{nav.y:.2f})\n")
            if nv<v-0.012: heading=(heading+random.uniform(70,290))%360
            elif nv>v+0.012: heading=tgt
            v=nv
            if v>0.75: mode="wall"; log.write("switch to wall-follow\n")
        else:
            # right-hand wall follow, continuous
            r=bot.ranges(); h=bot.heading()
            fr=[x for x in (r[0],r[1],r[15]) if x>=0]; front=min(fr) if fr else 9
            right=r[12] if r[12]>=0 else 9  # h+270
            rightf=r[14] if r[14]>=0 else 9 # h+315
            if front<0.28:
                bot.wheels(-20,60)  # rotate left
            elif right>0.7 and rightf>0.55:
                bot.wheels(70,15)   # turn right toward lost wall
            else:
                err=(0.38-right)
                corr=max(-18,min(18,err*80))
                bot.wheels(int(70-corr),int(70+corr))
            time.sleep(0.15)
            if not hasattr(main,'trail'): main.trail=[]
            main.trail.append((nav.x,nav.y))
            if len(main.trail)>400: main.trail.pop(0)
            if len(main.trail)==400:
                xs=[p[0] for p in main.trail]; ys=[p[1] for p in main.trail]
                if max(xs)-min(xs)+max(ys)-min(ys)<2.2:
                    log.write("island detected -> detach\n")
                    r2=bot.ranges(); h2=bot.heading()
                    j=max(range(16),key=lambda k:r2[k])
                    tgt2=(h2+22.5*j)%360
                    turn_to(bot,nav,tgt2)
                    drive(bot,nav,max(0.5,r2[j]-0.3),tgt2,speed=100,timeout=25)
                    main.trail=[]
            if time.time()-lastlog>2:
                lastlog=time.time()
                vs=[float(x) for x in bot.d11.last(3)]; vv=statistics.median(vs) if vs else -1
                log.write(f"wall t={time.time()-t0:4.0f} v={vv:.3f} pos=({nav.x:.2f},{nav.y:.2f}) fr={front:.2f} ri={right:.2f} st={st}\n")
                bot.tx(f"botB PING x={nav.x:.2f} y={nav.y:.2f} d11={vv:.3f} seq={seq}"); seq+=1
                if vv>0 and vv<0.62: mode="climb"; v=vv; log.write("back to climb\n")
    bot.stop()

if __name__=="__main__":
    main()
