import time, math, json, statistics
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

def scan_bearing(bot,nav,log):
    """slow spin, collect (heading,v), fit sinusoid, return bearing + stats"""
    data=[]
    bot.d11.take(); bot.comp.lines=[]
    bot.wheels(-10,10)  # spin increasing compass
    t0=time.time()
    while time.time()-t0<30:
        time.sleep(0.12)
        nav.update()
        h=bot.heading()
        vs=[float(x) for x in bot.d11.take() if x]
        if vs and h is not None: data.append((h,statistics.median(vs)))
        # stop after full revolution
        if time.time()-t0>6 and len(data)>20:
            hs=[d[0] for d in data]
            span=0
            # crude: check coverage
            covered=set(int(x//30) for x in hs)
            if len(covered)>=12: break
    bot.stop()
    # smoothed max: bin by 15 deg
    import math as m
    bins={}
    for h,v in data:
        b=int(h//15)
        bins.setdefault(b,[]).append(v)
    prof={b:sum(vs)/len(vs) for b,vs in bins.items()}
    if len(prof)<10: return None
    # smooth circularly over neighbors
    keys=sorted(prof)
    sm={}
    for b in range(24):
        vals=[prof.get((b+d)%24) for d in (-1,0,1)]
        vals=[v for v in vals if v is not None]
        sm[b]=sum(vals)/len(vals) if vals else -9
    bestb=max(sm,key=lambda b:sm[b])
    bearing=(bestb*15+7.5)%360
    a=sum(v for _,v in data)/len(data)
    amp=max(sm.values())-min(v for v in sm.values() if v>-9)
    log.write("profile "+" ".join(f"{b*15}:{sm[b]:.2f}" for b in sorted(sm))+"\n")
    return bearing,a,amp

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/home.log","a",buffering=1)
    t0=time.time(); seq=0
    while time.time()-t0<3000:
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        if rx: log.write(f"rx={rx[-3:]}\n")
        if st.get("here"):
            log.write(f"HERE! {st} pos=({nav.x:.2f},{nav.y:.2f})\n")
            bot.tx("botB: I AM AT GOAL (here=1). COME HERE.")
            bot.stop(); time.sleep(5); continue
        res=scan_bearing(bot,nav,log)
        if not res: continue
        bearing,base,amp=res
        # drive toward bearing, with obstacle avoidance, for a while
        leg_end=time.time()+ (8 if base>0.8 else 15)
        while time.time()<leg_end:
            nav.update()
            r=bot.ranges(); h=bot.heading()
            st=bot.status()
            if st.get("here"):
                log.write(f"HERE! {st} pos=({nav.x:.2f},{nav.y:.2f})\n"); bot.stop(); break
            # steer: choose clear beam nearest bearing
            best=None;bs=-1e9
            for i in range(16):
                if r[i]<0.35: continue
                beam=(h+22.5*i)%360
                sc=-abs(angdiff(beam,bearing))/45+min(r[i],1.2)
                if sc>bs: bs=sc;best=i
            if best is None:
                bot.wheels(-70,-70); time.sleep(1.2); bot.stop(); nav.update(); continue
            tgt=(h+22.5*best)%360
            d=angdiff(tgt,h)
            if abs(d)>25:
                turn_to(bot,nav,tgt); continue
            corr=max(-25,min(25,d*1.2))
            sp=90 if base<0.85 else 45
            bot.wheels(int(sp-corr),int(sp+corr))
            time.sleep(0.15)
        bot.stop()
        bot.tx(f"botB PING seq={seq} base={base:.2f}"); seq+=1
        log.write(f"leg done pos=({nav.x:.2f},{nav.y:.2f}) st={bot.status()}\n")
    bot.stop()

if __name__=="__main__":
    main()
