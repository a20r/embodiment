import time, math, json, statistics, random
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

def vmeas(bot,dur=1.2):
    bot.d11.take(); time.sleep(dur)
    vs=[float(x) for x in bot.d11.take() if x]
    return statistics.median(vs) if vs else -1

def rx_d11(rx):
    out=None
    for l in rx:
        if "botA" in l and "d11=" in l:
            try: out=float(l.split("d11=")[1].split()[0])
            except: pass
    return out

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/approach.log","a",buffering=1)
    t0=time.time(); seq=0
    heading=bot.heading()
    v=vmeas(bot)
    hist=[]
    lastplanmsg=0
    while time.time()-t0<2800:
        if time.time()-lastplanmsg>18:
            lastplanmsg=time.time()
            bot.tx("botB PLAN: botA lead me to goal: come to me til your d11>0.9, then move toward goal in small steps keeping my d11>0.8; stop+wait whenever my reported d11<0.8. I follow your signal.")
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        if rx: log.write(f"rx {rx[-1]}\n")
        if st.get("here"):
            bot.stop()
            log.write(f"HERE=1!!! t={time.time()-t0:.0f} pos=({nav.x:.2f},{nav.y:.2f}) st={st}\n")
            bot.tx("botB PING here=1 WE MADE IT, STAY PUT")
            time.sleep(2); seq+=1; continue
        hist.append((nav.x,nav.y))
        if len(hist)>8: hist.pop(0)
        if len(hist)==8 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.06:
            bot.wheels(-80,-80); e=time.time()+1.3
            while time.time()<e: time.sleep(0.1); nav.update()
            bot.stop(); heading=(heading+random.uniform(90,270))%360; hist=[]
            log.write("unstick\n")
        ad=rx_d11(rx)
        if ad is not None and ad>0.985:
            log.write(f"botA sees us at d11={ad}\n")
        # pick direction: continue current heading if clear, else best clear beam
        r=bot.ranges(); h=bot.heading()
        best=None;bs=-1e9
        for i in range(16):
            if r[i]<0.4: continue
            beam=(h+22.5*i)%360
            sc=-abs(angdiff(beam,heading))/60+min(r[i],1.0)
            if sc>bs: bs=sc;best=i
        if best is None:
            bot.wheels(-70,-70); time.sleep(1.0); bot.stop()
            heading=(heading+random.uniform(120,240))%360
            continue
        tgt=(h+22.5*best)%360
        turn_to(bot,nav,tgt)
        drive(bot,nav,0.25,tgt,speed=70)
        nv=vmeas(bot)
        bot.tx(f"botB PING x={nav.x:.2f} y={nav.y:.2f} d11={nv:.3f} seq={seq}"); seq+=1
        log.write(f"t={time.time()-t0:5.0f} v {v:.3f}->{nv:.3f} pos=({nav.x:.2f},{nav.y:.2f}) hd={heading:.0f} st={st}\n")
        if nv<v-0.01:
            heading=(heading+random.uniform(60,300))%360  # tumble
        elif nv>v+0.01:
            heading=tgt  # keep direction that worked
        v=nv
    bot.stop()

if __name__=="__main__":
    main()
