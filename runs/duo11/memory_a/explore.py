import rob, time, math, json, sys

LOG=open('/memory/map.log','a')
def log(**kw):
    kw['t']=round(time.time(),1)
    LOG.write(json.dumps(kw)+'\n'); LOG.flush()

K=0.010  # m per d9 count (rough)
pose=[0.0,0.0]

def d11():
    try: return float(rob.rd('d11'))
    except: return -1

def smooth_scan(n=2):
    ss=[rob.scan() for _ in range(n)]
    out=[]
    for i in range(16):
        vals=[s[i] for s in ss if s[i]>=0]
        out.append(sum(vals)/len(vals) if vals else 2.7)
    return out

def front(s): return min(s[0], s[1]*0.7+s[0]*0.3, s[15]*0.7+s[0]*0.3, s[0] if s[0]>0 else 2.7)

def drive_heading(H, dist, vmax=2):
    """drive holding compass heading H for dist meters or until blocked. returns meters moved, reason"""
    e0=rob.enc()[1]
    rob.drive(vmax)
    t0=time.time()
    reason='dist'
    while True:
        e=rob.enc()[1]
        moved=(e-e0)*K
        if moved>=dist: reason='dist'; break
        if time.time()-t0>dist/0.25+8: reason='timeout'; break
        s=rob.scan()
        f=min(x for x in (s[0],s[1],s[15]) if x>=0) if any(x>=0 for x in (s[0],s[1],s[15])) else 2.7
        if s[0]>=0 and s[0]<0.30: reason='blocked'; break
        if (s[1]>=0 and s[1]<0.18) or (s[15]>=0 and s[15]<0.18): reason='side'; break
        h=rob.heading(); err=rob.norm(H-h)
        rob.turn(max(min(-err*0.6,12),-12))
        time.sleep(0.15)
    rob.drive(0); rob.turn(0)
    moved=(rob.enc()[1]-e0)*K
    hr=math.radians(H)
    pose[0]+=moved*math.cos(hr); pose[1]+=moved*math.sin(hr)
    return moved, reason

def main():
    t_end=time.time()+float(sys.argv[1]) if len(sys.argv)>1 else time.time()+120
    h0=rob.heading()
    H=round(h0/90)*90 % 360   # grid headings
    n=0
    while time.time()<t_end:
        n+=1
        rob.tx(f"hello n={n}")
        s=smooth_scan()
        sig=d11()
        st=rob.status()
        log(pose=[round(pose[0],2),round(pose[1],2)], H=H, sig=sig, scan=[round(x,2) for x in s], st=st)
        # right-hand rule relative to compass: right = H+90 (beam4 side)
        def clear(i): 
            v=s[i]
            return v>0.55 or v<0
        # candidate order: right, straight, left, back
        cands=[(4,(H+90)%360),(0,H),(12,(H-90)%360),(8,(H+180)%360)]
        chosen=None
        for bi,newH in cands:
            if clear(bi): chosen=newH; break
        if chosen is None:
            chosen=(H+180)%360
        H=chosen
        rob.turn_to(H, tol=5)
        moved,reason=drive_heading(H, 0.45)
        log(move=round(moved,2), reason=reason, H=H)
    rob.drive(0); rob.turn(0)

main()
