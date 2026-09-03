import time, math, sys
sys.path.insert(0,'/bot/src')
exec(open('/bot/src/explore2.py').read().split("log('=== explore2 start ===')")[0].replace('log4','log6'))
# b starts at origin of THIS process; offset: botB told us in ctrl2-frame; ctrl2 frame died.
# Instead: climb d5 with target-direction = EAST bias initially; simpler: head east until d5>0.8 then pure d5 hill climb via short probes.
TX=6.0; TY=4.0  # rough, ctrl2 frame == this frame? no. We just go EAST.
def d5v():
    v=rdf('d5'); return v if v is not None else 0.0
log('=== goto east start ===')
mode_target=0.0  # east heading in compass deg? beams: world angle = heading+22.5k. East = 0 deg.
last=0
while True:
    l=lidar()
    if not l: continue
    v=d5v()
    if time.time()-last>2:
        log(f'pos=({b.x:.2f},{b.y:.2f}) d5={v:.2f}')
        wr('d0', f'botA coming, d5={v:.2f}')
        last=time.time()
    st=rd('d6')
    if check_events(st): break
    if v>0.93:
        drive(0,0); wr('d0','botA ARRIVED next to you. holding.'); log('arrived'); time.sleep(2); continue
    # choose open beam closest to east (or to d5 climb when close)
    best=None; bestsc=-9
    for k in range(16):
        r=min(l[k], l[(k+1)%16]+0.3, l[(k-1)%16]+0.3)
        if r<0.45: continue
        ang=(b.h+22.5*k)%360
        # score: alignment with east
        al=math.cos(math.radians(ang))
        sc=al*min(r,1.2)+ (0.3 if v>0.7 else 0)*0
        if sc>bestsc: bestsc=sc; best=k
    if best is None:
        drive(-30,-30); time.sleep(0.8); drive(0,0); continue
    t=(b.h+22.5*best)%360
    turn_to(t)
    leg(t, maxdist=0.8, sp=55)
