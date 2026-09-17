#!/usr/bin/env python3
# Autonomous explorer: gap-steering + d11 gradient ascent + periodic radio broadcast.
import json, time, math, sys, random
sys.path.insert(0,'/bot/src')
from rio import write, tx
from ctl import st, scan, hd, angdiff, load_pose, save_pose, stop, TPU, HeadingFilter
LOG='/tmp/explore.log'
def log(msg):
    with open(LOG,'a') as f: f.write(f"{time.time():.1f} {msg}\n")
def run(duration=90, mode='grad', pref_dir=None):
    hf=HeadingFilter(); pose=load_pose(); s=st()
    lp,rp=int(s['d9']),int(s['d6'])
    hist=[]  # (x,y,d11)
    visited={}  # cell->count
    t0=time.time(); lastlog=0; lasttx=0; target=None; lastretarget=0
    backing_until=0
    while time.time()-t0<duration:
        s=st(); h=hf.upd(hd(s))
        try: l=int(s['d9']); r=int(s['d6']); d11=float(s['d11'])
        except: time.sleep(0.02); continue
        d=((l-lp)+(r-rp))/2*TPU; lp,rp=l,r
        if h is None: time.sleep(0.02); continue
        pose['x']+=d*math.sin(math.radians(h)); pose['y']+=d*math.cos(math.radians(h))
        cell=(round(pose['x']/0.5),round(pose['y']/0.5)); visited[cell]=visited.get(cell,0)+1
        hist.append((pose['x'],pose['y'],d11)); hist=hist[-600:]
        st3=s.get('d3','')
        if 'goal=1' in st3 or 'here=1' in st3:
            stop(); log(f"STATUS CHANGE {st3} pose=({pose['x']:.2f},{pose['y']:.2f})"); save_pose(pose); return 'status'
        if s.get('d0')!='0' or s.get('d5')!='0': log(f"BUMP d0={s.get('d0')} d5={s.get('d5')}")
        sc=scan(s); sc=[v if v>=0 else 3.0 for v in sc]
        now=time.time()
        if now<backing_until:
            write('d1','-80'); write('d7','-80'); time.sleep(0.05); continue
        if min(sc[0],sc[1],sc[15])<0.13 or min(sc)<0.09:
            backing_until=now+0.6; log(f"escape: min={min(sc):.2f} at beam {sc.index(min(sc))}"); continue
        # desired direction
        want=None
        if mode=='grad' and len(hist)>80:
            xs=[p[0] for p in hist[-400:]]; ys=[p[1] for p in hist[-400:]]; zs=[p[2] for p in hist[-400:]]
            n=len(xs); mx=sum(xs)/n; my=sum(ys)/n; mz=sum(zs)/n
            sxx=sum((x-mx)**2 for x in xs); syy=sum((y-my)**2 for y in ys); sxy=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
            sxz=sum((x-mx)*(z-mz) for x,z in zip(xs,zs)); syz=sum((y-my)*(z-mz) for y,z in zip(ys,zs))
            det=sxx*syy-sxy*sxy
            if det>1e-4 and (sxx+syy)>0.05:
                bx=(sxz*syy-syz*sxy)/det; by=(syz*sxx-sxz*sxy)/det
                if math.hypot(bx,by)>0.05: want=math.degrees(math.atan2(bx,by))%360
        if pref_dir is not None and want is None: want=pref_dir
        # score beams
        if target is None or now-lastretarget>1.0:
            best=None;bs=-9
            for i in range(16):
                ang=(h+22.5*i)%360
                clr=min(sc[i], (sc[(i-1)%16]+sc[(i+1)%16])/2*1.3)
                if clr<0.35: continue
                sc_=min(clr,2.0)
                if want is not None: sc_+=1.5*math.cos(math.radians(angdiff(ang,want)))
                # novelty: cell 0.6 ahead in that direction
                cx=round((pose['x']+0.6*math.sin(math.radians(ang)))/0.5); cy=round((pose['y']+0.6*math.cos(math.radians(ang)))/0.5)
                sc_-=0.3*min(visited.get((cx,cy),0)/40,3)
                sc_-=0.4*abs(angdiff(ang,h))/180  # mild preference to keep going
                if sc_>bs: bs=sc_;best=ang
            target=best; lastretarget=now
        if target is None:
            write('d1','40'); write('d7','-40'); time.sleep(0.1); continue
        err=angdiff(target,h)
        # front clearance along current heading
        front=min(sc[0], sc[1]/0.92, sc[15]/0.92)
        if abs(err)>40 or front<0.28:
            sp=max(15,min(60,abs(err)))
            if err>0: write('d1',str(sp)); write('d7',str(-sp))
            else: write('d1',str(-sp)); write('d7',str(sp))
        else:
            v=max(50,min(180,(front-0.25)*300))
            corr=max(-50,min(50,err*2.0))
            write('d1',str(v+corr)); write('d7',str(v-corr))
        if now-lastlog>3:
            lastlog=now; save_pose(pose)
            log(f"pose=({pose['x']:.2f},{pose['y']:.2f}) hd={h:.0f} d11={d11:.3f} tgt={target:.0f} want={want if want is None else round(want)} front={front:.2f} min={min(sc):.2f} {st3}")
        if now-lasttx>20:
            lasttx=now; tx(f"Robot A here at approx ({pose['x']:.1f},{pose['y']:.1f}). Reply if you hear me.")
        time.sleep(0.05)
    stop(); save_pose(pose); log("done"); return 'done'
if __name__=="__main__":
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 90
    mode=sys.argv[2] if len(sys.argv)>2 else 'grad'
    pref=float(sys.argv[3]) if len(sys.argv)>3 else None
    print(run(dur,mode,pref))
