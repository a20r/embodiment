import time, math, sys
from robot import Robot
r=Robot(); time.sleep(0.4)
def beam(i):
    v=r.lidar[i%16]
    return 2.5 if v<0 else v
def turn_to(target):
    while True:
        h=r.hdg
        err=(target-h+180)%360-180
        if abs(err)<5: break
        s=max(5,min(35,abs(err)*0.7))
        r.motors(s if err>0 else -s, -s if err>0 else s)
        time.sleep(0.08)
    r.motors(0,0); time.sleep(0.2)
def drive(maxt=30):
    t0=time.time(); tgt=r.hdg
    dist=0; last=time.time()
    while time.time()-t0<maxt:
        now=time.time()
        front=min(beam(15),beam(0),beam(1))
        if front<0.28 or r.flags.get('d5')=='1':
            break
        # centering
        lft=beam(12); rgt=beam(4)
        steer=0
        if rgt<0.5 and lft<0.5:
            steer=max(-10,min(10,40*(rgt-lft)))
        h=r.hdg
        herr=(tgt-h+180)%360-180
        steer+=max(-15,min(15,herr*0.8))
        b=80 if front>0.6 else 40
        r.motors(b+steer,b-steer)
        dist+=0.0028*b*(now-last); last=now
        time.sleep(0.08)
    r.motors(0,0)
    return dist
def scan():
    pts={}
    r.motors(14,-14)
    t0=time.time()
    while time.time()-t0<9:
        h=r.hdg; l=r.lidar
        for i,v in enumerate(l):
            if v<0: continue
            a=int(round((h+22.5*i)%360/10)*10)%360
            pts.setdefault(a,[]).append(v)
        time.sleep(0.04)
    r.motors(0,0); time.sleep(0.2)
    out={}
    for a in sorted(pts):
        vs=sorted(pts[a]); out[a]=vs[len(vs)//2]
    return out
if __name__=="__main__":
    hdg=float(sys.argv[1])
    turn_to(hdg)
    d=drive()
    print(f"drove {d:.2f} at hdg {hdg}, stat={r.stat}")
    sc=scan()
    print("scan:", " ".join(f"{a}:{v:.2f}" for a,v in sc.items()))
