import rio, time, math
def clean(l): return [x if x>0 else 3.0 for x in l]
def turn_to(target_h, tol=8, tmax=6):
    t0=time.time()
    while time.time()-t0<tmax:
        h=rio.rdf('d1')
        if h is None: continue
        dh=(target_h-h+180)%360-180
        if abs(dh)<tol: rio.drive(0,0); return True
        s=max(12,min(35,int(abs(dh)*0.7)))
        rio.drive(s if dh>0 else -s, -s if dh>0 else s)
        time.sleep(0.05)
    rio.drive(0,0); return False
def drive_corridor(dur=20, sp=50, stop_front=0.30):
    t0=time.time(); target=None
    while time.time()-t0<dur:
        h=rio.rdf('d1'); l=rio.lidar()
        if h is None or not l: continue
        l=clean(l)
        if target is None: target=h
        f=min(l[0], l[1], l[15])
        if f<stop_front: rio.drive(0,0); return 'blocked'
        bal=0.0
        if min(l[4],l[12])<0.45:
            bal=(l[4]-l[12])*50
        dh=(target-h+180)%360-180
        turn=max(-30,min(30,dh*1.0+bal))
        rio.drive(int(sp+turn),int(sp-turn))
        time.sleep(0.08)
    rio.drive(0,0); return 'time'
def face_longest(exclude_back=False):
    l=rio.lidar(); h=rio.rdf('d1')
    if not l or h is None: return None
    l=clean(l)
    k=max(range(16), key=lambda i: min(l[i], l[(i+1)%16]+0.4, l[(i-1)%16]+0.4))
    target=(h+22.5*k)%360
    turn_to(target)
    return target
