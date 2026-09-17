def turnto(tgt,sp=24,timeout=4):
    t0=time.time()
    while time.time()-t0<timeout:
        h=R.heading()
        err=((tgt-h+180)%360)-180
        if abs(err)<7: break
        if err>0: R.motors(sp,-sp)
        else: R.motors(-sp,sp)
        time.sleep(0.07)
    R.stop(); time.sleep(0.12)

def driveguard(dur=2.6,sp=20):
    t0=time.time()
    while time.time()-t0<dur:
        l=R.lidar()
        if len(l)==16:
            f=min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
            if f<0.28: R.stop(); return False
        R.motors(sp,sp); time.sleep(0.06)
    R.stop(); return True

def pick(l,BAD):
    def b(i):
        v=l[i%16]; return v if v>0 else 2.5
    W={0:1.0,1:1.0,2:1.0,3:1.0,13:1.0,14:1.0,15:1.0,4:0.9,12:0.9,5:0.85,11:0.85,6:0.8,10:0.8,7:0.6,9:0.6,8:0.35}
    now=time.time()
    cands=[]
    for k in range(16):
        op=min(b(k),b((k-1)%16),b((k+1)%16))
        cands.append((op*W[k],k,op))
    cands.sort(reverse=True)
    for sc,k,op in cands:
        if BAD.get(k,0)<now-18: return k,op
    return cands[0][1],cands[0][2]

def wander():
    BAD={}
    n=0
    while True:
        n+=1
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        upd()
        GRID.add((int(pose['x']*2),int(pose['y']*2)))
        def b(i):
            v=l[i%16]; return v if v>0 else 2.5
        f=min(b(15),b(0),b(1))
        if f<0.18:
            R.motors(-14,-14); time.sleep(0.7); R.stop()
        k,op=pick(l,BAD)
        h=R.heading()
        tgt=(h+k*22.5)%360
        turnto(tgt)
        ok=driveguard(2.6,20)
        if not ok:
            BAD[k]=time.time()
        if n%12==0:
            s=sig(2,0.04)
            L('STEP pos=%.1f,%.1f k=%d op=%.2f ok=%s d11=%.3f tx=%s cells=%d'%(pose['x'],pose['y'],k,op,ok,s,txstate(),len(GRID)))
            SCAN.write('%.2f %.2f %s\n'%(pose['x'],pose['y'],','.join('%.2f'%v for v in l))); SCAN.flush()
