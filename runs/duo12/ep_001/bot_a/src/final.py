import time, math, statistics, random
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

def main():
    bot=Bot(); nav=Nav(bot)
    # sanity-cap Nav updates
    orig_update=nav.update
    def safe_update():
        e=bot.enc()
        d=((e[0]-nav.laste[0])+(e[1]-nav.laste[1]))/2*0.0006
        if abs(d)>0.08:
            nav.laste=e; return
        orig_update()
    nav.update=safe_update
    log=open("/memory/final.log","a",buffering=1)
    t0=time.time(); seq=0
    mode="climb"; heading=bot.heading()
    side=1  # 1=right-hand wall, -1=left-hand
    lastside=time.time()
    v=0.5; lastlog=0; lastmsg=0
    hist=[]
    def vv():
        vs=[float(x) for x in bot.d11.last(4)]
        return statistics.median(vs) if vs else -1
    while True:
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        for l in rx: log.write(f"rx {l}\n")
        if st.get("here"):
            bot.stop()
            log.write(f"HERE=1!!! t={time.time()-t0:.0f}\n")
            bot.tx("botB AT GOAL here=1. botA COME TO GOAL NOW, we must both be here within 60s!")
            time.sleep(1.0); continue
        if time.time()-lastmsg>25:
            lastmsg=time.time()
            bot.tx("botB REQUEST LEAD: botA (1) leave goal, approach me until your d11>0.93 (2) walk back to goal SLOWLY, 0.3/step, PAUSE whenever my ping d11<0.85 (3) park at goal. I will follow you through the entrance.")
            bot.tx(f"botB PING d11={vv():.3f} here=0 seq={seq}"); seq+=1
        nav.update()
        hist.append((nav.x,nav.y))
        if len(hist)>14: hist.pop(0)
        if len(hist)==14 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.04:
            bot.wheels(-85,-85); e=time.time()+1.2
            while time.time()<e: time.sleep(0.1); nav.update()
            bot.stop(); hist=[]
            heading=(bot.heading()+random.choice([90,-90,150]))%360
            log.write("unstick\n")
        # toggle wall side every 240s in wall mode
        if time.time()-lastside>240:
            lastside=time.time(); side=-side
            log.write(f"toggle side -> {side}\n")
        r=bot.ranges(); h=bot.heading()
        if r is None: time.sleep(0.1); continue
        cv=vv()
        if mode=="climb":
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
            drive(bot,nav,0.22 if v>0.85 else 0.5,tgt,speed=80 if v>0.85 else 95)
            nv=vv()
            if nv<v-0.012: heading=(heading+random.uniform(70,290))%360
            elif nv>v+0.012: heading=tgt
            v=nv
            if time.time()-lastlog>3:
                lastlog=time.time()
                log.write(f"climb t={time.time()-t0:4.0f} v={v:.3f} pos=({nav.x:.2f},{nav.y:.2f}) st={st}\n")
            if v>0.75: mode="wall"; lastside=time.time(); log.write("-> wall\n")
        else:
            fr=[x for x in (r[0],r[1],r[15]) if x>=0]; front=min(fr) if fr else 9
            if side==1:
                wallb=r[12] if r[12]>=0 else 9; wallf=r[14] if r[14]>=0 else 9
            else:
                wallb=r[4] if r[4]>=0 else 9; wallf=r[2] if r[2]>=0 else 9
            # gap-shoot: strong opening roughly toward v-uphill? just any big beam next to tight walls
            gap=None
            for i in range(16):
                if r[i]>1.3 and r[(i-1)%16]<0.4 and r[(i+1)%16]<0.4:
                    gap=i; break
            if gap is not None and cv>0.7:
                tgt=(h+22.5*gap)%360
                log.write(f"gap-shoot beam{gap} v={cv:.3f}\n")
                turn_to(bot,nav,tgt,tol=5)
                drive(bot,nav,min(r[gap]-0.25,1.2),tgt,speed=45,timeout=30)
                continue
            if front<0.26:
                bot.wheels(-20*side if side==1 else 60,60 if side==1 else -20)
                # rotate away from wall side
                if side==1: bot.wheels(-25,60)
                else: bot.wheels(60,-25)
            elif wallb>0.7 and wallf>0.55:
                if side==1: bot.wheels(70,12)
                else: bot.wheels(12,70)
            else:
                err=(0.38-wallb)*side
                corr=max(-18,min(18,err*80))
                bot.wheels(int(68-corr),int(68+corr))
            time.sleep(0.15)
            if time.time()-lastlog>3:
                lastlog=time.time()
                log.write(f"wall t={time.time()-t0:4.0f} side={side} v={cv:.3f} pos=({nav.x:.2f},{nav.y:.2f}) fr={front:.2f} st={st}\n")
            if cv>0 and cv<0.6: mode="climb"; v=cv; log.write("-> climb\n")

if __name__=="__main__":
    main()
