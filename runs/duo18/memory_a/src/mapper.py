#!/usr/bin/env python3
# Frontier-based exploration with occupancy grid (pure python). Stops on goal=1/here=1.
import json, time, math, sys, os, collections
sys.path.insert(0,'/bot/src')
from rio import write
from ctl import st, scan, hd, angdiff, load_pose, save_pose, stop, TPU, HeadingFilter
RES=0.12; N=220; H=N//2   # world = +-12.8 units
LOG='/tmp/map.log'; MAXR=2.8; INFL=1
def log(m):
    with open(LOG,'a') as f: f.write(f"{time.time():.1f} {m}\n")
def c2i(x,y): return int(round(x/RES))+H, int(round(y/RES))+H
def i2c(i,j): return (i-H)*RES, (j-H)*RES
class Grid:
    def __init__(s):
        s.o=[0]*(N*N)
        if os.path.exists('/tmp/grid.json'):
            try: s.o=json.load(open('/tmp/grid.json'))
            except: pass
    def save(s): json.dump(s.o, open('/tmp/grid.json','w'))
    def upd(s,i,j,v):
        if 0<=i<N and 0<=j<N:
            k=i*N+j; s.o[k]=max(-12,min(12,s.o[k]+v))
    def integrate(s,x,y,h,sc):
        for b,r in enumerate(sc):
            if r<0: continue
            ang=math.radians(h+22.5*b); hit=r<MAXR; r=min(r,MAXR)
            dx=math.sin(ang); dy=math.cos(ang)
            n=int(r/RES); last=None
            for k in range(1,n):
                i,j=c2i(x+dx*k*RES,y+dy*k*RES)
                if (i,j)!=last: s.upd(i,j,-1); last=(i,j)
            if hit:
                i,j=c2i(x+dx*r,y+dy*r); s.upd(i,j,3)
    def occ(s,i,j): return s.o[i*N+j]>2
    def free(s,i,j): return s.o[i*N+j]<-1
    def safe(s,i,j):
        inf=s.infl if hasattr(s,'infl') else INFL
        for di in range(-inf,inf+1):
            for dj in range(-inf,inf+1):
                ii,jj=i+di,j+dj
                if 0<=ii<N and 0<=jj<N and s.o[ii*N+jj]>2: return False
        return True
    def plan(s,si,sj,blacklist):
        # BFS from start over safe free cells; return path to nearest frontier cell
        prev={ (si,sj):None }; q=collections.deque([(si,sj)]); best=None
        while q:
            i,j=q.popleft()
            d=abs(i-si)+abs(j-sj)
            # frontier test: free cell adjacent to unknown
            if d>3 and (i,j) not in blacklist and s.free(i,j):
                for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                    k=(i+di)*N+(j+dj)
                    if -2<=s.o[k]<=2 and not s.occ(i+di,j+dj): best=(i,j); break
                if best: break
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                ni,nj=i+di,j+dj
                if 0<ni<N-1 and 0<nj<N-1 and (ni,nj) not in prev and not s.occ(ni,nj) and s.safe(ni,nj):
                    prev[(ni,nj)]=(i,j); q.append((ni,nj))
        if not best: return None
        path=[]; c=best
        while c: path.append(c); c=prev[c]
        return path[::-1]
    def plan_to(s,si,sj,ti,tj):
        prev={(si,sj):None}; q=collections.deque([(si,sj)]); best=(si,sj); bd=abs(si-ti)+abs(sj-tj)
        while q:
            i,j=q.popleft(); d=abs(i-ti)+abs(j-tj)
            if d<bd: bd=d; best=(i,j)
            if d==0: break
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                ni,nj=i+di,j+dj
                if 0<ni<N-1 and 0<nj<N-1 and (ni,nj) not in prev and not s.occ(ni,nj) and s.safe(ni,nj):
                    prev[(ni,nj)]=(i,j); q.append((ni,nj))
        path=[]; c=best
        while c: path.append(c); c=prev[c]
        return path[::-1], bd
    def ascii(s,x,y):
        i0,j0=c2i(x,y); out=[]
        for j in range(j0+40,j0-41,-2):
            row=''
            for i in range(i0-50,i0+51,2):
                if abs(i-i0)<=1 and abs(j-j0)<=1: row+='R'; continue
                v=s.o[i*N+j] if 0<=i<N and 0<=j<N else 0
                row+='#' if v>2 else ('.' if v<-1 else ' ')
            out.append(row)
        return "\n".join(out)
def run(duration, goto=None, goto_explore_after=False, convoy=False):
    g=Grid(); hf=HeadingFilter(); pose=load_pose(); s=st(); lp,rp=int(s['d9']),int(s['d6'])
    t0=time.time(); lastlog=0; lastplan=0; lastsave=0; path=[]; wp=None; blacklist=set(); wp_t=0; k=0
    rxseen=len(open('/tmp/rx.log').read().splitlines()) if os.path.exists('/tmp/rx.log') else 0
    back_until=0; stuck_wp=0; samples=[]; lastsample=0; mode=('home' if os.environ.get('HOME_MODE')=='1' else 'explore')
    while time.time()-t0<duration:
        s=st(); h=hf.upd(hd(s)); k+=1; now=time.time()
        try: l=int(s['d9']); r=int(s['d6'])
        except: time.sleep(0.02); continue
        d=((l-lp)+(r-rp))/2*TPU; lp,rp=l,r
        if h is None: time.sleep(0.02); continue
        pose['x']+=d*math.sin(math.radians(h)); pose['y']+=d*math.cos(math.radians(h))
        st3=s.get('d3','')
        if 'goal=1' in st3:
            stop(); save_pose(pose); g.save(); log(f"STATUS {st3} pose=({pose['x']:.2f},{pose['y']:.2f})")
            open('/tmp/beacon_extra.txt','w').write("*** goal=1 *** I REACHED THE GOAL and am PARKED AT THE GOAL. Come to me: home in on your d11 (rises toward me). I stay here.")
            return 'goal'
        if 'here=1' in st3: log(f"HERE flag set: {st3}")
        if now-lastsample>0.5:
            lastsample=now
            try: samples.append((pose['x'],pose['y'],float(s['d11']))); samples=samples[-60:]
            except: pass
        now=time.time(); sc=scan(s)
        if k%3==0: g.integrate(pose['x'],pose['y'],h,sc)
        scc=[v if v>=0 else 3.0 for v in sc]
        bump=s.get('d0')!='0' or s.get('d5')!='0'
        if now<back_until: write('d1','-70'); write('d7','-70'); time.sleep(0.05); continue
        if bump or min(scc)<0.09:
            back_until=now+0.5; path=[]; wp=None; log(f"escape bump={bump} min={min(scc):.2f}"); continue
        if os.path.exists('/tmp/rx.log'):
            rl=open('/tmp/rx.log').read().splitlines()
            if len(rl)>rxseen:
                for m in rl[rxseen:]:
                    log("RX: "+m[:200]); mu=m.upper()
                    if (('GOAL=1' in mu.replace(' ','')) or 'REACHED THE GOAL' in mu) and 'GOAL=0' not in mu.replace(' ',''):
                        if mode!='home': log('partner at goal -> home mode')
                        mode='home'
                    if 'RESUMED' in mu or 'MOVING AGAIN' in mu: mode='explore'
                rxseen=len(rl)
        # convoy: park if partner signal weak
        try: d11v=float(s['d11'])
        except: d11v=1.0
        d11s=0.9*globals().get('_d11s',d11v)+0.1*d11v; globals()['_d11s']=d11s
        if convoy and mode=='explore':
            waiting=globals().get('_wait',False); wt0=globals().get('_wt0',now)
            if not waiting and d11s<0.72:
                globals()['_wait']=True; globals()['_wt0']=now; stop(); log(f"convoy: partner weak d11={d11s:.2f}, parking")
                open('/tmp/beacon_extra.txt','w').write("LEADER PARKED waiting for you - home in on my signal (your d11 up). I resume when you are close.")
                continue
            if waiting:
                if d11s>0.85 or now-wt0>240:
                    globals()['_wait']=False; log(f"convoy: resuming d11={d11s:.2f} waited {now-wt0:.0f}s")
                    open('/tmp/beacon_extra.txt','w').write("LEADER MOVING & exploring; follow my signal. I park when you fall behind.")
                else:
                    stop(); time.sleep(0.2); continue
        # planning
        if wp is None or now-lastplan>8:
            si,sj=c2i(pose['x'],pose['y'])
            for di in (-1,0,1):
                for dj in (-1,0,1): g.upd(si+di,sj+dj,-6)   # robot cell is free
            if mode=='home' and len(samples)>=8:
                sm=samples[-30:]; n=len(sm); mx=sum(a[0] for a in sm)/n; my=sum(a[1] for a in sm)/n; mz=sum(a[2] for a in sm)/n
                sxx=sum((a[0]-mx)**2 for a in sm); syy=sum((a[1]-my)**2 for a in sm); sxy=sum((a[0]-mx)*(a[1]-my) for a in sm)
                sxz=sum((a[0]-mx)*(a[2]-mz) for a in sm); syz=sum((a[1]-my)*(a[2]-mz) for a in sm); det=sxx*syy-sxy*sxy
                if det>1e-4 and sxx+syy>0.05:
                    bx=(sxz*syy-syz*sxy)/det; by=(syz*sxx-sxz*sxy)/det; gn=math.hypot(bx,by)
                    if gn>0.03:
                        goto=(pose['x']+0.8*bx/gn, pose['y']+0.8*by/gn); log(f"home: grad dir {math.degrees(math.atan2(bx,by))%360:.0f} d11={sm[-1][2]:.3f}")
                    else: goto=None
                else: goto=None
            elif mode=='home': goto=None
            if goto is not None:
                ti,tj=c2i(*goto); p,bd=g.plan_to(si,sj,ti,tj)
                if len(p)<3 or math.hypot(goto[0]-pose['x'],goto[1]-pose['y'])<0.15:
                    if mode!='home' and not goto_explore_after:
                        stop(); save_pose(pose); g.save(); log(f"goto reached/closest bd={bd} pose=({pose['x']:.2f},{pose['y']:.2f})"); return 'goto'
                    log(f"goto target unreachable/reached (bd={bd}); frontier instead"); p=g.plan(si,sj,blacklist)
            else:
                g.infl=1; p=g.plan(si,sj,blacklist)
                if p is None: g.infl=0; p=g.plan(si,sj,blacklist); log("plan with infl=0")
            lastplan=now
            if p is None:
                log("no frontier found; rotating"); write('d1','40'); write('d7','-40'); time.sleep(0.5); stop(); blacklist=set(); continue
            # waypoints every 4 cells
            path=[i2c(*c) for c in p[3::4]]+[i2c(*p[-1])]; wp=path.pop(0); wp_t=now; goalcell=p[-1]
        # waypoint following
        dx=wp[0]-pose['x']; dy=wp[1]-pose['y']; dist=math.hypot(dx,dy)
        if dist<0.10:
            if path: wp=path.pop(0); wp_t=now
            else: blacklist.add(goalcell); wp=None; continue
            continue
        if now-wp_t>12:   # stuck on this waypoint
            log(f"wp timeout at ({pose['x']:.2f},{pose['y']:.2f}) -> blacklist"); blacklist.add(goalcell); wp=None; stuck_wp+=1
            back_until=now+0.4; continue
        tgt=math.degrees(math.atan2(dx,dy))%360; err=angdiff(tgt,h)
        front=min(scc[0],scc[1]/0.92,scc[15]/0.92)
        if abs(err)>35:
            sp=max(15,min(60,abs(err)))
            write('d1',str(sp if err>0 else -sp)); write('d7',str(-sp if err>0 else sp))
        elif front<0.22:
            # blocked: mark and replan
            log(f"blocked front={front:.2f}"); wp=None; back_until=now+0.5
            for bb in (0,1,15):
                if 0<=scc[bb]<1.0:
                    ang=math.radians(h+22.5*bb); ci,cj=c2i(pose['x']+math.sin(ang)*(scc[bb]+0.05),pose['y']+math.cos(ang)*(scc[bb]+0.05)); g.upd(ci,cj,12)
        else:
            v=max(50,min(150,(front-0.2)*400, dist*400+40)); corr=max(-45,min(45,err*2.0))
            write('d1',str(v+corr)); write('d7',str(v-corr))
        if now-lastlog>4:
            lastlog=now; save_pose(pose)
            log(f"[{mode}] pose=({pose['x']:.2f},{pose['y']:.2f}) hd={h:.0f} d11={s.get('d11')} wp={None if wp is None else (round(wp[0],2),round(wp[1],2))} left={len(path)} bl={len(blacklist)} {st3}")
        if now-lastsave>20: lastsave=now; g.save(); open('/tmp/map.txt','w').write(g.ascii(pose['x'],pose['y']))
        time.sleep(0.05)
    stop(); save_pose(pose); g.save(); open('/tmp/map.txt','w').write(g.ascii(pose['x'],pose['y'])); log("done"); return 'done'
if __name__=="__main__":
    goto=(float(sys.argv[2]),float(sys.argv[3])) if len(sys.argv)>3 else None
    print(run(float(sys.argv[1]) if len(sys.argv)>1 else 120, goto, goto_explore_after=(len(sys.argv)>4), convoy=(os.environ.get('CONVOY')=='1')))
