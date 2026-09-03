import time, math, sys
sys.path.insert(0,'/bot/src')
from lib import rd, wr, drive, stop, tx

LOG='/tmp/nav.log'
def log(m):
    with open(LOG,'a') as f: f.write('%.1f %s\n'%(time.time(),m))

def lidar(prev=[None]):
    while True:
        try:
            v=[float(x) for x in rd('d3').split(',')]
            if len(v)==16:
                if prev[0]:
                    v=[v[i] if v[i]>=0 else prev[0][i] for i in range(16)]
                else:
                    v=[x if x>=0 else 3.0 for x in v]
                prev[0]=v
                return v
        except: pass
def heading():
    while True:
        try: return float(rd('d1'))
        except: pass
def goal():
    try: return int(rd('d6').split('goal=')[1].split()[0])
    except: return 0
def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d
def ray_at(l,h,ang):
    i=int(round(((ang-h)%360)/22.5))%16
    return l[i]

def turn_to(target):
    t0=time.time()
    while time.time()-t0<12:
        h=heading()
        e=angdiff(target,h)
        if abs(e)<4:
            stop(); time.sleep(0.12)
            if abs(angdiff(target,heading()))<6: return
            continue
        s=max(12,min(55,abs(e)*1.1))
        drive(int(s if e>0 else -s), int(-s if e>0 else s))
        time.sleep(0.04)
    stop()

X=[0.0]; Y=[0.0]
K=0.0065
def integrate(cl,cr,dt,h):
    v=K*(cl+cr)/2.0
    hr=math.radians(h)
    X[0]+=v*dt*math.cos(hr); Y[0]+=v*dt*math.sin(hr)

def straight(target_h, maxtime=10):
    t0=time.time(); tprev=t0
    l=lidar(); h=heading()
    r0=ray_at(l,h,(target_h+90)%360); l0=ray_at(l,h,(target_h-90)%360)
    right_closed=r0<0.45; left_closed=l0<0.45
    traveled=0.0; cl=cr=0
    while True:
        now=time.time(); dt=now-tprev; tprev=now
        h=heading()
        integrate(cl,cr,dt,h)
        traveled+=K*(cl+cr)/2.0*dt
        l=lidar()
        if goal(): stop(); return 'GOAL',traveled
        front=ray_at(l,h,target_h)
        fr=min(ray_at(l,h,(target_h+22.5)%360), ray_at(l,h,(target_h+45)%360))
        fl=min(ray_at(l,h,(target_h-22.5)%360), ray_at(l,h,(target_h-45)%360))
        r=ray_at(l,h,(target_h+90)%360)
        lf=ray_at(l,h,(target_h-90)%360)
        try: bump=rd('d9')=='1'
        except: bump=False
        if bump:
            drive(-40,-40); time.sleep(0.5); stop()
            return 'bump',traveled
        if front<0.24 or min(fr,fl)<0.12: stop(); return 'blocked',traveled
        if right_closed and r>0.6 and traveled>0.25: stop(); return 'open_right',traveled
        if left_closed and lf>0.6 and traveled>0.25: stop(); return 'open_left',traveled
        if not right_closed and r<0.45: right_closed=True
        if not left_closed and lf<0.45: left_closed=True
        if now-t0>maxtime: stop(); return 'timeout',traveled
        e=angdiff(target_h,h)
        cen=(r-lf)*40 if (r<0.5 and lf<0.5) else 0.0
        rep=-9.0*(1.0/max(fr,0.06)-1.0/max(fl,0.06))
        rep=max(-30,min(30,rep))
        steer=max(-32,min(32,e*1.5+cen+rep))
        base=65 if front>0.5 else 35
        cl=int(base+steer); cr=int(base-steer)
        drive(cl,cr)
        time.sleep(0.06)

def backup(d=0.2):
    drive(-45,-45); time.sleep(d/(K*45)); stop(); time.sleep(0.1)

visits={}
def cellkey():
    return (round(X[0]*2)/2, round(Y[0]*2)/2)

def choose(cur):
    stop(); time.sleep(0.15)
    l=lidar(); h=heading()
    ck=cellkey()
    cands=[]  # (visits, rank, -dist, world_angle)
    for rank,rel in enumerate([90,0,-90,180]):  # right, straight, left, back
        p=(cur+rel)%360
        # rays within +-33.75 of p
        best=None
        for i in range(16):
            wa=(h+22.5*i)%360
            if abs(angdiff(wa,p))<=34:
                if best is None or l[i]>l[best]: best=i
        if best is None: continue
        d=l[best]; wa=(h+22.5*best)%360
        if d>0.55:
            vk=(ck, round(wa/45))
            cands.append((visits.get(vk,0), rank, -d, wa, vk))
    if not cands:
        return None,None
    cands.sort()
    _,_,_,wa,vk=cands[0]
    return wa,vk


import threading, random
sig={'v':None}
def d5_thread():
    buf=[]; last=time.time()
    with open('/dev/robot/d5') as f:
        while True:
            line=f.readline().strip()
            if line:
                try: buf.append(float(line))
                except: pass
            now=time.time()
            if now-last>0.8 and buf:
                sig['v']=sum(buf)/len(buf); buf=[]; last=now
threading.Thread(target=d5_thread,daemon=True).start()

def sample(dur=1.2):
    stop(); time.sleep(dur)
    return sig['v']

def main():
    l=lidar(); h=heading()
    best=max(range(16),key=lambda i:l[i])
    cur=(h+22.5*best)%360
    prev_s=sample()
    log('homing2 start s=%.3f'%(prev_s or -1))
    last_g=0
    momentum=0  # >0 keep straight
    while True:
        now=time.time()
        if goal():
            stop()
            if now-last_g>8:
                last_g=now; tx('GOALFOUND from=beta (standing on goal)'); log('GOAL!')
            time.sleep(0.3); continue
        wa,vk=choose(cur)
        if wa is None:
            wa=(cur+180)%360; vk=(cellkey(),round(wa/45))
        turn_to(wa)
        lch=lidar(); h=heading()
        if ray_at(lch,h,wa)<0.3:
            visits[vk]=visits.get(vk,0)+2
            continue
        visits[vk]=visits.get(vk,0)+1
        cur=wa
        mt=1.8 if (prev_s is not None and prev_s>0.6) else 4
        r,trav=straight(cur, maxtime=mt)
        s1=sample(1.0)
        d=(s1-prev_s) if (s1 is not None and prev_s is not None) else 0
        log('leg %.0f %s trav=%.2f s=%.3f d=%+.3f'%(cur,r,trav,s1 or -1,d))
        tx('PING from=beta homing d5=%.2f'%(s1 or -1))
        if d<-0.025 and trav>0.3:
            # got worse: turn around, and penalize this dir
            visits[vk]=visits.get(vk,0)+3
            cur=(cur+180)%360
            log('worse -> reverse to %.0f'%cur)
        prev_s=s1
main()
