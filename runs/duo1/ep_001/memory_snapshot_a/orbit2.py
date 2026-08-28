import rob, time, walker

bng=walker.bng; serr=walker.serr; turn=walker.turn; align=walker.align; step=walker.step; look=walker.look

def try_enter():
    b=bng()
    L=rob.lidar()
    gb=(16-int(round((b%360)/22.5)))%16
    for i in (gb, (gb+1)%16, (gb-1)%16):
        if L[i]>0.5:
            # face that beam then drive
            turn(-serr(b - 22.5*i if False else 0,0))  # noop placeholder
    return False

def main():
    t0=time.time()
    while time.time()-t0<3300:
        if rob.goal(): print("GOAL!!!", flush=True); return
        align()
        b=bng()
        L=rob.lidar()
        gb=(16-int(round((b%360)/22.5)))%16
        # ENTER if goalward beam deep
        cand_enter=[]
        for i in (gb,(gb+1)%16,(gb-1)%16):
            if L[i]>0.5: cand_enter.append(i)
        if cand_enter:
            i=cand_enter[0]
            # face beam i: reading must become b - ... facing beam i changes reading from b to b-22.5*i? 
            # empirona: facing beam i means rotating by +22.5*i?? test: face_beam in ctl used target=(h+22.5*i) turn_to i.e. reading increases
            turn(22.5*i if 22.5*i<=180 else 22.5*i-360)
            b2=bng()
            print(f"ENTER attempt beam{i} b now {b2:.0f}", flush=True)
            if abs(serr(b2,0))<25:
                r=step()
                print(f"  entered step {r} b={bng():.0f} F={rob.lidar()[0]:.2f}", flush=True)
                if rob.goal(): print("GOAL!!!", flush=True); return
                continue
            # else fall through to orbit
        F,R,Lt,B=look()
        b=bng()
        opts=[]
        for dd,dist in ((0,F),(90,R),(-90,Lt),(180,B)):
            if dist<0.38: continue
            nb=(b+dd)%360  # reading after turn... turn(dd) => reading+dd
            score=abs(serr(nb,90))
            opts.append((score,dd,dist))
        if not opts:
            print("boxed in!", flush=True); turn(180); continue
        opts.sort()
        _,dd,dist=opts[0]
        if dd: turn(dd if dd!=180 else 180)
        r=step()
        print(f"orbit b={b:.0f} chose {dd} dist={dist:.2f} step={r} nb={bng():.0f}", flush=True)
    print("timeout", flush=True)

if __name__=="__main__":
    main()
