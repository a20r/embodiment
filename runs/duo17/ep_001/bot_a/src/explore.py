import sys,time,math,json,os,statistics as st
sys.path.insert(0,'/bot/src'); import ctl as c, rio as m
K=0.0007  # units per count
LOG=open('/bot/src/explore.log','a'); MAP=open('/bot/src/map.jsonl','a')
def log(*a):
    s=time.strftime('%H:%M:%S ')+' '.join(str(x) for x in a); LOG.write(s+'\n'); LOG.flush()
AX=85.0  # maze axis
def snap_axis(h): return (AX+90*round(((h-AX)%360)/90))%360
class Pose:
    def __init__(s):
        s.x=0; s.y=0
        try:
            d=json.load(open('/bot/src/pose.json')); s.x=d['x']; s.y=d['y']
        except Exception: pass
        a,b=c.enc(); s.a=a; s.b=b; s.h=c.heading() or 0; s.hs=[s.h]
    def upd(s):
        a,b=c.enc(); h=c.heading()
        if a is None or b is None: return
        if h is not None:
            s.hs.append(h); s.hs=s.hs[-3:]
            # circular median approx: use last value smoothed
            s.h=h
        d=((a-s.a)+(b-s.b))/2*K; s.a=a; s.b=b
        s.x+=d*math.sin(math.radians(s.h)); s.y+=d*math.cos(math.radians(s.h))
P=Pose()
def lid():
    for _ in range(3):
        L=c.lidar()
        if L: return L
    return None
_rn=[0]
def record(L):
    _rn[0]+=1
    if _rn[0]%3: return
    MAP.write(json.dumps({'t':round(time.time(),2),'x':round(P.x,3),'y':round(P.y,3),'h':round(P.h,1),'L':L})+'\n'); MAP.flush()
last_ping=0; pingn=0; last_d11=None
def housekeeping(force=False):
    global last_ping,pingn
    now=time.time()
    if now-last_ping>2.0 or force:
        last_ping=now; pingn+=1
        m.wr('d8','PING from robot A. pos x=%.2f y=%.2f (my odom frame). If you hear this, reply!'%(P.x,P.y),0.3)
        stt=m.rd('d3'); rx=m.rd('d10'); d11=m.rd('d11'); d0=m.rd('d0'); d5=m.rd('d5')
        log('POSE x=%.2f y=%.2f h=%.0f | %s | d11=%s d0=%s d5=%s | rx=%r'%(P.x,P.y,P.h,stt,d11,d0,d5,rx))
        alert=False
        if stt and ('lost' not in stt or 'goal=0' not in stt or 'here=0' not in stt): alert=True
        if rx: alert=True
        if d0 not in ('0',None) or d5 not in ('0',None): log('BUMP? d0=%s d5=%s'%(d0,d5))
        try: json.dump({'x':P.x,'y':P.y,'h':P.h,'t':time.time()},open('/bot/src/pose.json','w'))
        except Exception as e: pass
        if alert:
            log('!!! ALERT: status=%s rx=%r d0=%s d5=%s -> STOPPING for operator'%(stt,rx,d0,d5)); c.stop()
            open('/bot/src/ALERT','w').write('%s\n%s\n%s\n'%(stt,rx,'d0=%s d5=%s'%(d0,d5)))
            return True
    return False
def move_forward(counts, axis, spd=70, wall_follow=True, min_front=0.28):
    a0,b0=c.enc(); t0=time.time()
    while time.time()-t0<counts/(spd*5)+3:
        P.upd(); a,b=c.enc()
        if a is None: continue
        if ((a-a0)+(b-b0))/2>=counts: break
        L=lid()
        if not L: continue
        record(L)
        F=L[0]
        if F<0: continue
        if F<min_front: log('front block %.2f'%F); break
        h=P.h; e=c.angdiff(axis,h); corr=max(-20,min(20,e*1.2))
        if wall_follow and 0<L[12]<0.45 and 0<L[4]<0.45:
            corr-=max(-10,min(10,(L[12]-L[4])*40))  # left(L12) bigger -> steer left (heading decrease) -> corr negative
        elif wall_follow and 0<L[12]<0.45:
            corr-=max(-10,min(10,(L[12]-0.25)*40))
        c.drive(spd+corr, spd-corr)
        if housekeeping(): return 'alert'
        time.sleep(0.02)
    c.stop(); return 'ok'
def turn(axis):
    h=c.turn_to(axis, tol=4, spd=45); P.upd(); log('turned to %.0f (target %.0f)'%(h,axis)); return h
def main():
    axis=snap_axis(P.h); turn(axis); rule=-1  # -1: left-hand (prefer heading decrease)
    visits={}
    while True:
        P.upd(); L=lid()
        if not L: continue
        record(L)
        cell=(round(P.x/0.5),round(P.y/0.5)); visits[cell]=visits.get(cell,0)+1
        if visits[cell]>6 and visits[cell]%7==0:
            rule=-rule; log('cell %s visited %d times -> switching rule to %d'%(cell,visits[cell],rule))
        F=L[0]
        if F<0: continue
        Lside=L[12]; Lf=L[13]; R=L[4]; Rf=L[3]
        pref_side = Lside if rule==-1 else R
        pref_f = Lf if rule==-1 else Rf
        other_side = R if rule==-1 else Lside
        log('DECIDE x=%.2f y=%.2f h=%.0f axis=%.0f F=%.2f L=%.2f Lf=%.2f R=%.2f Rf=%.2f rule=%d'%(P.x,P.y,P.h,axis,F,Lside,Lf,R,Rf,rule))
        if pref_side>0.7 and pref_f>0.55:
            # opening on preferred side: advance to center of it then turn
            r=move_forward(int(0.25/K), axis, spd=60, wall_follow=False)
            if r=='alert': return
            axis=(axis+90*rule)%360; turn(axis)
            r=move_forward(int(0.35/K), axis, spd=60, wall_follow=False)
            if r=='alert': return
        elif F>0.45:
            r=move_forward(int(0.3/K), axis)
            if r=='alert': return
        elif other_side>0.6:
            axis=(axis-90*rule)%360; turn(axis)
        else:
            axis=(axis+180)%360; turn(axis)
        if housekeeping(): return
if __name__=='__main__':
    try:
        log('=== explore start'); main()
    finally:
        c.stop(); log('=== explore end')
