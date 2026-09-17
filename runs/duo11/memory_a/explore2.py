import rob2 as R, time, math, json, sys, random

LOG=open('/memory/map2.log','a')
def log(**kw):
    kw['t']=round(time.time(),1)
    LOG.write(json.dumps(kw)+'\n'); LOG.flush()

pose=[0.0,0.0]  # x=east-ish (cos h), y (sin h) -- compass frame
def upd(H, dist):
    r=math.radians(H)
    pose[0]+=dist*math.cos(r); pose[1]+=dist*math.sin(r)

def main():
    tend=time.time()+ (float(sys.argv[1]) if len(sys.argv)>1 else 240)
    n=0
    lastH=None
    while time.time()<tend:
        n+=1
        s=R.scan(); h=R.heading()
        sg=R.sig(); st=R.status()
        R.tx(f"HELLO from bot at step {n} sig={sg}")
        log(pose=[round(pose[0],2),round(pose[1],2)], h=round(h,1), sig=sg, st=st,
            scan=[round(x,2) for x in s])
        if 'here=1' in st or 'goal=1' in st:
            log(event='STATUS FLAG!', st=st)
        # choose direction: grid headings; prefer continuing straight, then most open
        def beam_for(H):  # beam index pointing at compass H
            return int(round(((H-h)%360)/22.5))%16
        cands=[0,90,180,270]
        best=None; bestscore=-9
        for H in cands:
            bi=beam_for(H)
            d=[s[(bi+k)%16] for k in (-1,0,1)]
            d=[x if x>0 else 2.7 for x in d]
            openness=min(d[1], max(d[0],d[2])+9)  # mostly center beam
            score=openness+ (0.4 if lastH==H else 0) + random.uniform(0,0.3)
            if d[1]<0.5: score-=5
            if lastH is not None and H==(lastH+180)%360: score-=1.0
            if score>bestscore: bestscore=score; best=H
        H=best
        R.turn_to(H)
        b0s,b0e,reason=R.fwd_step(H, maxtime=7)
        moved=max(0.0,(b0s-b0e)) if (b0s<2.65 and b0e<2.65) else 0.19*min(7,7)
        # if b0 capped, estimate by time
        upd(H, moved)
        log(H=H, moved=round(moved,2), reason=reason, b0s=b0s, b0e=b0e)
        if reason=='stuck':
            R.unstick(H)
            lastH=None
        else:
            lastH=H
    R.stop()

main()
