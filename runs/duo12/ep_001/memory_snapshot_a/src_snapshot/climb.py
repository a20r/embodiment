import time, math, json, statistics
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

def vmeas(bot,dur=2.5):
    time.sleep(0.3); bot.d11.take()
    time.sleep(dur)
    vs=[float(x) for x in bot.d11.take() if x]
    return statistics.median(vs) if vs else -1

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/climb.log","a",buffering=1)
    cur=vmeas(bot)
    log.write(f"start v={cur:.3f} pos=({nav.x:.2f},{nav.y:.2f})\n")
    fails=0
    for it in range(60):
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        if rx: log.write(f"rx={rx}\n")
        if st.get("here") or st.get("goal"):
            log.write(f"FLAG {st}\n")
        h=bot.heading(); r=bot.ranges()
        # candidate directions: clear beams
        cands=[i for i in range(16) if r[i]>0.4]
        if not cands:
            bot.wheels(-60,-60); time.sleep(1); bot.stop(); continue
        # probe: pick direction with best local gradient by sampling move
        # simple: move along each of up to 4 spread beams 0.12, measure, keep if better
        improved=False
        cands.sort(key=lambda i: -r[i])
        tried=0
        for i in cands:
            if tried>=4: break
            beam=(h+22.5*i)%360
            tried+=1
            turn_to(bot,nav,beam)
            drive(bot,nav,0.15,beam,speed=60)
            nv=vmeas(bot)
            log.write(f"it{it} beam{i} v {cur:.3f}->{nv:.3f} pos=({nav.x:.2f},{nav.y:.2f}) st={bot.status()}\n")
            if nv>cur+0.005:
                cur=nv; improved=True; break
            else:
                back=(beam+180)%360
                turn_to(bot,nav,back)
                drive(bot,nav,0.15,back,speed=60)
        if not improved:
            fails+=1
            cur=vmeas(bot)
            if fails>=3:
                log.write(f"converged v={cur:.3f} pos=({nav.x:.2f},{nav.y:.2f}) st={bot.status()}\n")
                fails=0
        else: fails=0
    bot.stop()

if __name__=="__main__":
    main()
