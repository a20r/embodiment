import sys,time,math,json,os
sys.path.insert(0,'/bot/src'); import ctl as c, rio as m
K=0.0007; AX=90.0
LOG=open('/bot/src/explore.log','a'); MAP=open('/bot/src/map2.jsonl','a')
def log(*a):
    s=time.strftime('%H:%M:%S ')+' '.join(str(x) for x in a); LOG.write(s+'\n'); LOG.flush()
def snap_axis(h): return (AX+90*round(((h-AX)%360)/90))%360
def fl(x):
    try: return float(x)
    except: return None
class Odo:
    def __init__(s):
        s.x=s.y=0.0
        try: d=json.load(open('/bot/src/pose.json')); s.x=d['x']; s.y=d['y']
        except Exception: pass
        s.a,s.b=c.enc(); s.h=c.heading() or 0.0; s.F=s.B=None; s.L=None; s.n=0
    def upd(s):
        a,b=c.enc(); h=c.heading(); L=c.lidar()
        if a is None or b is None: return
        if h is not None: s.h=h
        de=((a-s.a)+(b-s.b))/2*K; turn=abs((a-s.a)-(b-s.b))>25; s.a=a; s.b=b
        d=de
        if L:
            F=L[0] if L[0]>0 else None; B=L[8] if L[8]>0 else None
            if not turn and abs(de)>0.001:
                cands=[]
                if F and s.F: cands.append(s.F-F)
                if B and s.B: cands.append(B-s.B)
                sg=1 if de>0 else -1
                ok=[v for v in cands if -0.02<=v*sg<=abs(de)+0.02]
                if ok: d=sg*max(0.0,min(abs(de),min(v*sg for v in ok)))
            s.F=F; s.B=B; s.L=L
        s.x+=d*math.sin(math.radians(s.h)); s.y+=d*math.cos(math.radians(s.h))
        s.n+=1
        if L and s.n%3==0:
            MAP.write(json.dumps({'x':round(s.x,3),'y':round(s.y,3),'h':round(s.h,1),'L':L})+'\n'); MAP.flush()
O=Odo()
def d11():
    v=[fl(m.rd('d11')) for _ in range(3)]; v=[x for x in v if x is not None]
    return sum(v)/len(v) if v else None
def save_pose(): json.dump({'x':O.x,'y':O.y,'h':O.h,'t':time.time()},open('/bot/src/pose.json','w'))
last_tx=0
def radio(msg=None):
    """send ping (rate limited), check status/rx. returns True if contact."""
    global last_tx
    now=time.time()
    if now-last_tx>2.2:
        last_tx=now
        m.wr('d8', msg or ('A: homing on you. my d11=%.2f heading=%.0f x=%.1f y=%.1f'%(d11() or -1, O.h, O.x, O.y)),0.3)
    stt=m.rd('d3') or ''; rx=m.rd('d10')
    if rx: log('RX <<< %r'%rx); open('/bot/src/RX.txt','a').write(rx+'\n')
    return ('ok' in stt) or bool(rx), stt, rx
def contact_protocol():
    c.stop(); v=d11()
    log('*** CONTACT. starting handshake'); 
    msg=('Robot A here. IMPORTANT: a float sensor (mine is /dev/robot/d11, ~0.3-0.7) is a GOAL BEACON: it INCREASES toward the goal. '
         'Follow increasing values. Mine now=%.2f, compass heading %.0f. PLAN: both climb the beacon. Whoever arrives first waits AT the goal '
         'and pings every 2s; the other homes in. Reply with your beacon value + heading.'%(v or -1,O.h))
    got=[]; t0=time.time(); last=0
    while time.time()-t0<25:
        if time.time()-last>2.5: m.wr('d8',msg,0.3); last=time.time()
        rx=m.rd('d10'); stt=m.rd('d3')
        if rx: got.append(rx); log('RX <<< %r'%rx); open('/bot/src/RX.txt','a').write(rx+'\n')
        time.sleep(0.3)
    log('handshake end status=%s got=%d msgs'%(stt,len(got)))
    if got:
        open('/bot/src/ALERT','w').write('\n'.join(got)); return True
    return False
def front(): return O.L[0] if O.L and O.L[0]>0 else 9
def steer(axis, spd):
    L=O.L; tgt=axis
    if L:
        l=L[12]; r=L[4]
        if 0<l<0.5 and 0<r<0.5: tgt=axis+max(-22,min(22,(r-l)*70))
        elif 0<l<0.5: tgt=axis+max(-22,min(22,(0.22-l)*70))
        elif 0<r<0.5: tgt=axis-max(-22,min(22,(0.22-r)*70))
    e=c.angdiff(tgt,O.h); corr=max(-25,min(25,e*1.3))
    c.drive(spd+corr,spd-corr)
def go(dist, axis, spd=70, min_front=0.27, stop_on_side=None):
    """move dist (neg=reverse) along axis w/ centering; returns reason"""
    x0,y0=O.x,O.y; sg=1 if dist>0 else -1; t0=time.time(); trav=0
    while time.time()-t0<abs(dist)/(spd*5*K)+4:
        O.upd(); trav=math.hypot(O.x-x0,O.y-y0)
        if trav>=abs(dist): c.stop(); return 'done'
        if sg>0 and front()<min_front: c.stop(); return 'wall'
        if sg<0 and O.L and 0<O.L[8]<0.22: c.stop(); return 'wall'
        if stop_on_side and O.L:
            if (stop_on_side in ('L','both')) and O.L[12]>0.7 and O.L[13]>0.5: c.stop(); return 'openL'
            if (stop_on_side in ('R','both')) and O.L[4]>0.7 and O.L[3]>0.5: c.stop(); return 'openR'
        if sg>0: steer(axis,spd)
        else:
            e=c.angdiff(axis,O.h); corr=max(-20,min(20,e*1.3)); c.drive(-spd+corr,-spd-corr)
        ok,stt,rx=radio()
        if rx and 'HELLO robot A from robot B' not in rx: log('NEW MSG -> writing ALERT'); open('/bot/src/ALERT','a').write(rx+'\n')
        if 'here=1' in stt or 'goal=1' in stt:
            c.stop(); log('!!! STATUS %s -> STOP'%stt); open('/bot/src/ALERT','a').write('STATUS '+stt+'\n'); return 'alert'
        time.sleep(0.02)
    c.stop(); return 'timeout'
def turn(axis):
    h=c.turn_to(axis,tol=4,spd=45); O.upd(); return h
def trial(axis_branch, cur_axis):
    turn(axis_branch); r=go(0.45,axis_branch,spd=60)
    if r=='alert': raise SystemExit
    time.sleep(0.2); v=d11(); log('  trial axis %.0f -> %s beacon=%.3f'%(axis_branch,r,v or -1))
    go(-0.45,axis_branch,spd=60); turn(cur_axis)
    return v
visited={}
def jkey(): return (round(O.x/0.5),round(O.y/0.5))
def main():
    axis=snap_axis(O.h); turn(axis); O.upd()
    while True:
        O.upd(); save_pose()
        if not O.L: continue
        L=O.L; F=front()
        opts={}
        if F>0.5: opts['S']=axis
        if L[12]>0.65: opts['L']=(axis-90)%360
        if L[4]>0.65: opts['R']=(axis+90)%360
        v=d11()
        if False:
            c.stop(); plan="A PLAN: both explore the maze independently for the GOAL (status goal=1?). Whoever finds it STAYS there, pings 'GOAL' every 5s with d11; other homes in by RSSI. I start exploring now. Keep msgs <250 chars."
            for i in range(5):
                m.wr('d8',plan); time.sleep(2.5); stt=m.rd('d3') or ''; rx=m.rd('d10')
                if rx: log('RX <<< %r'%rx); open('/bot/src/ALERT','a').write(rx+'\n')
                log('plan send %d: d11=%.3f %s'%(i,d11() or -1,stt))
                if 'ok' in stt: break
            log('PLAN DELIVERED? %s -> exiting for operator'%stt); open('/bot/src/ALERT','a').write('PLAN '+stt+'\n'); return
        log('AT x=%.2f y=%.2f h=%.0f axis=%.0f F=%.2f L=%.2f R=%.2f beacon=%.3f opts=%s'%(O.x,O.y,O.h,axis,F,L[12],L[4],v or -1,list(opts)))
        if not opts:
            axis=(axis+180)%360; turn(axis); continue
        if len(opts)==1:
            k=list(opts)[0]; axis=opts[k]
            if k!='S': turn(axis); go(0.3,axis)
        else:
            key=jkey(); tried=visited.setdefault(key,{})
            for k,ax in opts.items():
                if ax not in tried: tried[ax]=trial(ax,axis) or 0
            # choose best untried-by-travel branch: highest beacon not yet 'taken'
            taken=visited.setdefault(('taken',)+key,set())
            cand=[(val,ax) for ax,val in tried.items() if ax in opts.values() and ax not in taken]
            if not cand: cand=[(val,ax) for ax,val in tried.items() if ax in opts.values()]
            cand.sort(reverse=True); best=cand[0][1]; taken.add(best)
            log('  junction %s tried=%s -> go axis %.0f'%(key,{int(a):round(b,3) for a,b in tried.items()},best))
            if best!=axis: axis=best; turn(axis)
            go(0.3,axis)
        # cruise along axis until wall or side opening; then position at junction center
        r=go(3.0,axis,stop_on_side='both')
        stt=m.rd('d3') or ''
        if 'here=1' in stt or 'goal=1' in stt: log('!!! STATUS %s'%stt); open('/bot/src/ALERT','a').write('STATUS '+stt+'\n'); return
        log('  cruise -> %s at x=%.2f y=%.2f'%(r,O.x,O.y))
        if r=='alert': return
        if r in ('openL','openR'): go(0.22,axis,min_front=0.2)
if __name__=='__main__':
    try: log('=== explore2 start'); main()
    finally: c.stop(); save_pose(); log('=== explore2 end')
