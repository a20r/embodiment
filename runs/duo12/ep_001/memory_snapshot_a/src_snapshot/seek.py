import time, math, json, sys
from robot import Bot, angdiff
from explore import Nav, turn_to, drive

def d11val(bot,n=6,dt=0.15):
    vs=[]
    end=time.time()+n*dt
    while time.time()<end:
        vs+= [float(x) for x in bot.d11.last(2)]
        time.sleep(dt)
    vs=sorted(set(vs))
    return sum(vs)/len(vs)

def main():
    bot=Bot(); nav=Nav(bot)
    log=open("/memory/seek.log","a",buffering=1)
    samples=[]  # (x,y,val)
    seq=0; t0=time.time()
    target_h=None
    while time.time()-t0<3000:
        nav.update()
        v=d11val(bot)
        samples.append((nav.x,nav.y,v))
        if len(samples)>40: samples.pop(0)
        st=bot.status()
        rx=[l for l in bot.rx.take() if l.strip()]
        log.write(json.dumps({"t":round(time.time()-t0,1),"x":round(nav.x,3),"y":round(nav.y,3),
          "v":round(v,3),"st":st,"rx":rx})+"\n")
        bot.tx(f"botB PING x={nav.x:.2f} y={nav.y:.2f} d11={v:.3f} seq={seq}"); seq+=1
        if st.get("here")==1:
            log.write("HERE=1 !!! stopping\n"); bot.stop(); time.sleep(3); continue
        # estimate gradient from recent samples with spatial spread
        gx=gy=None
        pts=[p for p in samples]
        if len(pts)>=8:
            n=len(pts)
            sx=sum(p[0] for p in pts); sy=sum(p[1] for p in pts); sz=sum(p[2] for p in pts)
            sxx=sum(p[0]**2 for p in pts); syy=sum(p[1]**2 for p in pts); sxy=sum(p[0]*p[1] for p in pts)
            sxz=sum(p[0]*p[2] for p in pts); syz=sum(p[1]*p[2] for p in pts)
            try:
                det=(sxx*(syy*n-sy*sy)-sxy*(sxy*n-sy*sx)+sx*(sxy*sy-syy*sx))
                if abs(det)>1e-9:
                    gx=(sxz*(syy*n-sy*sy)-sxy*(syz*n-sy*sz)+sx*(syz*sy-syy*sz))/det
                    gy=(sxx*(syz*n-sz*sy)-sxz*(sxy*n-sy*sx)+sx*(sxy*sz-syz*sx))/det
            except Exception: pass
        if gx is not None and math.hypot(gx,gy)>0.02:
            target_h=math.degrees(math.atan2(gy,gx))%360
        elif target_h is None:
            target_h=314.0  # prior from global fit
        # pick clear beam nearest target_h
        r=bot.ranges(); h=bot.heading()
        best=None;bestscore=-1e9
        for i in range(16):
            beam=(h+22.5*i)%360
            if r[i]<0.45: continue
            score=-abs(angdiff(beam,target_h))/45 + min(r[i],1.0)
            if score>bestscore: bestscore=score;best=i
        if best is None:
            turn_to(bot,nav,(h+180)%360); 
            drive(bot,nav,0.3,(h+180)%360,speed=60)
            continue
        beam=(h+22.5*best)%360
        turn_to(bot,nav,beam)
        drive(bot,nav,max(0.2,min(r[best]-0.35,0.6)),beam,speed=80)
    bot.stop()

if __name__=="__main__":
    main()
