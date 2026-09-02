import time, math, json, statistics, random, re
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

def vmeas(bot,dur=1.0):
    bot.d11.take(); time.sleep(dur)
    vs=[float(x) for x in bot.d11.take() if x]
    return statistics.median(vs) if vs else -1

def fit(pts):
    best=None
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    cx=sum(xs)/len(xs); cy=sum(ys)/len(ys)
    for sx in [cx+dx*0.25 for dx in range(-24,25)]:
        for sy in [cy+dy*0.25 for dy in range(-24,25)]:
            ds=[math.hypot(p[0]-sx,p[1]-sy) for p in pts]
            ls=[math.log(max(p[2],1e-3)) for p in pts]
            n=len(pts); sd=sum(ds); sl=sum(ls); sdd=sum(d*d for d in ds); sdl=sum(d*l for d,l in zip(ds,ls))
            den=n*sdd-sd*sd
            if abs(den)<1e-9: continue
            slope=(n*sdl-sd*sl)/den; inter=(sl-slope*sd)/n
            err=sum((l-(inter+slope*d))**2 for d,l in zip(ds,ls))
            if slope<-0.05 and (best is None or err<best[0]): best=(err,sx,sy)
    return best

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/goto.log","a",buffering=1)
    pts=[]
    # seed from approach.log history
    for l in open("/memory/approach.log"):
        m=re.match(r"t=\s*\d+ v [\d.]+->([\d.]+) pos=\((-?[\d.]+),(-?[\d.]+)\)",l)
        if m: pts.append((float(m.group(2)),float(m.group(3)),float(m.group(1))))
    pts=pts[-150:]
    t0=time.time(); seq=0; lastfit=0; src=None
    hist=[]
    while time.time()-t0<2700:
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        if st.get("here"):
            bot.stop()
            log.write(f"HERE=1! pos=({nav.x:.2f},{nav.y:.2f})\n")
            bot.tx("botB here=1 STAY PUT")
            time.sleep(2); continue
        hist.append((nav.x,nav.y))
        if len(hist)>7: hist.pop(0)
        if len(hist)==7 and max(abs(hist[-1][0]-p[0])+abs(hist[-1][1]-p[1]) for p in hist)<0.06:
            bot.wheels(-85,-85); e=time.time()+1.4
            while time.time()<e: time.sleep(0.1); nav.update()
            bot.stop(); hist=[]
            log.write("unstick\n")
            import random as _r
            turn_to(bot,nav,(bot.heading()+_r.choice([70,-70,140]))%360)
        v=vmeas(bot,0.8)
        pts.append((nav.x,nav.y,v)); pts=pts[-200:]
        if time.time()-lastfit>15:
            lastfit=time.time()
            f=fit(pts)
            if f: src=(f[1],f[2]); log.write(f"fit src=({src[0]:.2f},{src[1]:.2f}) err={f[0]:.3f} pos=({nav.x:.2f},{nav.y:.2f}) v={v:.3f}\n")
        bot.tx(f"botB PING x={nav.x:.2f} y={nav.y:.2f} d11={v:.3f} seq={seq}"); seq+=1
        if rx: log.write(f"rx {rx[-1]}\n")
        if src is None: time.sleep(1); continue
        bearing=math.degrees(math.atan2(src[1]-nav.y,src[0]-nav.x))%360
        r=bot.ranges(); h=bot.heading()
        best=None;bs=-1e9
        for i in range(16):
            if r[i]<0.35: continue
            beam=(h+22.5*i)%360
            sc=-abs(angdiff(beam,bearing))/45+min(r[i],1.2)
            if sc>bs: bs=sc;best=i
        if best is None:
            bot.wheels(-70,-70); time.sleep(1.1); bot.stop(); continue
        tgt=(h+22.5*best)%360
        turn_to(bot,nav,tgt)
        drive(bot,nav,min(0.5,max(0.2,math.hypot(src[0]-nav.x,src[1]-nav.y)/2)),tgt,speed=80)
        nav.update()
        log.write(f"t={time.time()-t0:4.0f} v={v:.3f} pos=({nav.x:.2f},{nav.y:.2f}) brg={bearing:.0f}\n")
    bot.stop()

if __name__=="__main__":
    main()
