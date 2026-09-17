import sys, time, math, pickle, os
sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
STEP=0.62          # target advance per cell
VIS=0.35          # visited cell size
class DFS:
    def __init__(self):
        self.r=R()
        self.x=0.0; self.y=0.0; self.mpt=0.000503; self.prev=None
        self.visited={}         # coarse cell -> time
        self.blocked={}         # (cell,oct) -> count
        self.wallmap={}         # passive wall log for map drawing
        self.trail=[]
        self.logf=open('/memory/dfs.log','a',buffering=1)
        self.rxlog=open('/memory/rx.log','a',buffering=1)
        self.last_ping=0
        self.n=0
        self.found=False
        self.goal_xy=None
        self.gseen=False
        self.zone=None
        self.zonetime=0
        self.zang=0
        try:
            import json
            j=json.load(open('/memory/zone.json'))
            self.zone=(j['x'],j['y']); self.zonetime=time.time()
            self.log(f"loaded zone {self.zone}")
        except Exception: pass
        try:
            import json
            j=json.load(open('/memory/pose.json'))
            if time.time()-j.get('t',0)<180:
                self.x=j['x']; self.y=j['y']
                self.log(f"restored pose {self.x},{self.y}")
        except Exception: pass
    def log(self,s): self.logf.write(f"[{time.time():.0f}] {s}\n")
    def vkey(self,x,y): return (int(math.floor(x/VIS)),int(math.floor(y/VIS)))
    def enc(self):
        for _ in range(4):
            e=self.r.enc()
            if e[0] is not None and e[1] is not None: return e
            time.sleep(0.03)
        return None
    def odo(self,h,e):
        if e and self.prev and h is not None:
            d=((e[0]-self.prev[0])+(e[1]-self.prev[1]))/2.0*self.mpt
            self.x+=d*math.cos(math.radians(h)); self.y+=d*math.sin(math.radians(h))
        if e: self.prev=e
    def ping(self,msg=None):
        now=time.time()
        if msg: self.r.write(8,msg)
        elif now-self.last_ping>3.5:
            self.last_ping=now
            self.r.write(8,f"B2 PING x={self.x:.2f} y={self.y:.2f} t={now:.0f}")
        m=self.r.read(10,0.1)
        if m:
            self.rxlog.write(f"[{time.time():.0f}] RX: {m}\n")
            if 'GOAL' in m.upper() or 'ATGOAL' in m.upper():
                self.log(f"!! RX: {m}")
                try:
                    parts=dict(p.split('=',1) for p in m.split() if '=' in p)
                    if 'x' in parts and 'y' in parts:
                        self.goal_xy=(float(parts['x']),float(parts['y']))
                        self.log(f"goal_xy={self.goal_xy}")
                except: pass
            return m
        return None
    def turnto(self,target,tol=3.0):
        r=self.r
        for _ in range(80):
            h=r.heading()
            if h is None: time.sleep(0.05); continue
            e=wrap(target-h)
            if abs(e)<tol: r.motors(0,0); time.sleep(0.1); return True
            v=int(max(-90,min(90,2.2*e)))
            r.motors(v,-v); time.sleep(0.08)
        r.motors(0,0); return False
    def goto(self,tx,ty,maxt=7.0):
        r=self.r; t0=time.time()
        res='ok'
        while time.time()-t0<maxt:
            st=r.status() or (0,0,0)
            if st[2]==1: return 'here'
            rg=r.ranges()
            if rg is None: r.motors(0,0); time.sleep(0.06); continue
            f0=rg[0] if (rg[0] is not None and rg[0]>=0) else 9
            side=[rg[i] for i in (1,15) if rg[i] is not None and rg[i]>=0]
            smin=min(side) if side else 9
            if f0<0.28 or smin<0.16:
                r.motors(-60,-60); time.sleep(0.5); r.motors(0,0); return 'bump'
            if f0<0.40:
                r.motors(0,0); return 'wall'
            if math.hypot(tx-self.x,ty-self.y)<0.15:
                r.motors(0,0); return 'ok'
            h=r.heading(); e=self.enc()
            if h is None:
                r.motors(0,0); time.sleep(0.06); continue
            self.odo(h,e)
            ang=math.degrees(math.atan2(ty-self.y,tx-self.x))
            err=wrap(ang-h)
            base=65 if f0>0.8 else 45
            a=int(max(-50,min(100, base+1.6*err)))
            b=int(max(-50,min(100, base-1.6*err)))
            r.motors(a,b); time.sleep(0.09)
        r.motors(0,0)
        return 'timeout'
    def wall_stamp(self,rg,h):
        for k,val in enumerate(rg):
            if val is None or val<0 or val>1.9: continue
            a=math.radians(h+k*22.5)
            wx=self.x+val*math.cos(a); wy=self.y+val*math.sin(a)
            key=(int(round(wx/0.1)),int(round(wy/0.1)))
            c=self.wallmap.get(key,0)
            self.wallmap[key]=min(c+1,5)
    def choose(self):
        rg=self.r.ranges()
        if not rg: return None
        h=self.r.heading()
        if h is None: return None
        cands=[]
        for k in range(16):
            v=rg[k]
            if v is None or v<0 or v<0.55: continue
            ang=h+k*22.5
            tx=self.x+STEP*math.cos(math.radians(ang))
            ty=self.y+STEP*math.sin(math.radians(ang))
            vk=self.vkey(tx,ty)
            oct_=int(round(((ang%360)/45)))%8
            bl=self.blocked.get((vk,oct_),0)
            vt=self.visited.get(vk)
            score=v*1.0 + (60 if vt is None else 0) - bl*40
            if vt is not None: score -= min(30,(time.time()-vt)/10.0)
            cands.append((score,k,ang,tx,ty,v,vk,vt))
        if not cands: return None
        cands.sort(reverse=True)
        return cands[0]
    def run(self):
        r=self.r
        self.log("DFS START")
        mode='explore'; idle=0
        while True:
            open('/memory/heartbeat','w').write(f"{time.time():.0f} {self.x:.2f} {self.y:.2f} m={mode}\n")
            if os.path.exists('/memory/STOP'):
                r.motors(0,0)
                import json
                json.dump({'x':self.x,'y':self.y,'t':time.time()},open('/memory/pose.json','w'))
                self.log("STOP FILE"); break
            # external request channel
            if os.path.exists('/memory/request.json'):
                try:
                    import json
                    rq=json.load(open('/memory/request.json'))
                    os.remove('/memory/request.json')
                    if 'goto' in rq:
                        gx,gy=rq['goto']
                        self.log(f"REQUEST goto ({gx:.2f},{gy:.2f})")
                        self.turnto(math.degrees(math.atan2(gy-self.y,gx-self.x)),tol=4)
                        res=self.goto(gx,gy,maxt=20)
                        self.log(f"REQUEST res={res}")
                except Exception as ee:
                    self.log(f"req err {ee!r}")
            st=r.status() or (0,0,0)
            tick,goal,here=st
            if here==1:
                if not self.found:
                    self.found=True
                    self.log("*** HERE=1 AT GOAL ***")
                    pickle.dump({'x':self.x,'y':self.y,'h':r.heading()},open('/memory/goalfound.pkl','wb'))
                r.motors(0,0)
                self.ping(f"B2 ATGOAL x={self.x:.2f} y={self.y:.2f}")
                time.sleep(1.0)
                continue
            if goal==1:
                if not self.gseen:
                    self.gseen=True
                    self.log(f"*** goal=1 FIRST TIME at ({self.x:.2f},{self.y:.2f},{r.heading():.0f}) ***")
                    self.ping(f"B2 GOALSEEN x={self.x:.2f} y={self.y:.2f}")
                self.log(f"goal=1 at ({self.x:.2f},{self.y:.2f})")
                # local sweep: small steps spiral
                h2=r.heading() or 0
                self.turnto(h2+40,tol=5)
                res=self.goto(self.x+0.35*math.cos(math.radians(h2+40)),self.y+0.35*math.sin(math.radians(h2+40)),maxt=4)
                self.ping(f"B2 GOALSEEN x={self.x:.2f} y={self.y:.2f}")
                continue
            d0=r.read(0); d5=r.read(5)
            if (d0 and d0 not in ('0','-')) or (d5 and d5 not in ('0','-')):
                self.log(f"SENSOR d0={d0} d5={d5} at ({self.x:.2f},{self.y:.2f})")
                import json
                json.dump({'x':self.x,'y':self.y},open('/memory/zone.json','w'))
                json.dump({'x':self.x,'y':self.y,'t':time.time()},open('/memory/pose.json','w'))
                self.zone=(self.x,self.y); self.zonetime=time.time()
            # persistent GOAL navigation (BFS over wallmap)
            if os.path.exists('/memory/goal.json'):
                try:
                    import json as _js
                    from collections import deque as _dq
                    gj=_js.load(open('/memory/goal.json'))
                    gx,gy=gj['x'],gj['y']
                    d=math.hypot(gx-self.x,gy-self.y)
                    if st[2]==1 or d<0.18:
                        r.motors(0,0)
                        self.ping(f"B2 ATGOAL x={self.x:.2f} y={self.y:.2f}")
                        self.log(f"AT GOAL TARGET ({self.x:.2f},{self.y:.2f}) here={st[2]}")
                        time.sleep(0.7)
                        continue
                    # fresh scan + stamp before planning
                    _rg=r.ranges(); _h=r.heading()
                    if _rg is not None and _h is not None:
                        self.odo(_h,self.enc())
                        self.wall_stamp(_rg,_h)
                        time.sleep(0.12)
                    # build coarse grid 0.1m from wallmap
                    G={}
                    for (wi,wj),c in self.wallmap.items():
                        if c>=2:
                            G[(wi,wj)]=1
                            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                                G[(wi+di,wj+dj)]=1
                    start=(int(round(self.x/0.1)),int(round(self.y/0.1)))
                    goalc=(int(round(gx/0.1)),int(round(gy/0.1)))
                    def bfs(start,goalc):
                        seen={start:None}; q=_dq([start]); N=0
                        while q:
                            cur=q.popleft(); N+=1
                            if N>15000: break
                            if cur==goalc:
                                path=[]
                                while cur!=start:
                                    path.append(cur); cur=seen[cur]
                                return path[::-1]
                            ci,cj=cur
                            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                                nk=(ci+di,cj+dj)
                                if nk in seen or G.get(nk,0)==1: continue
                                seen[nk]=cur; q.append(nk)
                        return None
                    path=bfs(start,goalc)
                    if path is None:
                        # nearest reachable to goal
                        best=None;bd=1e9
                        for cell in seen:
                            dd=(cell[0]-goalc[0])**2+(cell[1]-goalc[1])**2
                            if dd<bd: bd=dd;best=cell
                        if best and bd>0:
                            path=bfs(start,best)
                            self.log(f"goal unreachable; nearest {best}")
                    if path:
                        # take waypoints up to ~0.55m
                        wp=[start]
                        for c in path:
                            wp.append(c)
                            if math.hypot((c[0]-start[0])*0.1,(c[1]-start[1])*0.1)>0.55: break
                        tgt=wp[-1]
                        tx,ty=tgt[0]*0.1,tgt[1]*0.1
                        ang=math.degrees(math.atan2(ty-self.y,tx-self.x))
                        self.turnto(ang,tol=5)
                        res=self.goto(tx,ty,maxt=6)
                        self.log(f"GOALBFS d={d:.2f} path={len(path)} wp={tgt} res={res}")
                        if res in ('wall','bump'):
                            bx=self.x+0.3*math.cos(math.radians(ang)); by=self.y+0.3*math.sin(math.radians(ang))
                            bk=(int(round(bx/0.1)),int(round(by/0.1)))
                            self.wallmap[bk]=self.wallmap.get(bk,0)+3
                            self.log(f" penalize {bk}")
                    else:
                        self.log("BFS FAIL - sidestep")
                        h3=r.heading() or 0
                        self.turnto(h3+75,tol=5)
                        self.goto(self.x+0.35*math.cos(math.radians(h3+75)),self.y+0.35*math.sin(math.radians(h3+75)),maxt=3)
                    self.ping(f"B2 GOALIS x=0.40 y=-0.80")
                    continue
                except Exception as _ge:
                    self.log(f"goalnav err {_ge!r}")
            # GOAL ZONE SEARCH
            if self.zone and (time.time()-self.zonetime)<30:
                self.ping(f"B2 GOALSEEN x={self.x:.2f} y={self.y:.2f}")
                dxz=self.zone[0]-self.x; dyz=self.zone[1]-self.y
                dz=math.hypot(dxz,dyz)
                if dz>0.85:
                    ang=math.degrees(math.atan2(dyz,dxz))
                else:
                    self.zang+=55
                    ang=self.zang
                self.log(f"ZONESCAN d=({dz:.2f}) ang={ang%360:.0f}")
                self.turnto(ang,tol=5)
                step=0.3 if dz<=0.85 else min(dz-0.3,0.5)
                res=self.goto(self.x+step*math.cos(math.radians(ang)),self.y+step*math.sin(math.radians(ang)),maxt=4)
                continue
            self.ping()
            # scan while stopped
            r.motors(0,0); time.sleep(0.22)
            h=r.heading(); e=self.enc(); rg=r.ranges()
            if h is None or e is None or rg is None:
                time.sleep(0.1); continue
            self.odo(h,e)
            self.wall_stamp(rg,h)
            self.n+=1
            self.trail.append((round(self.x,2),round(self.y,2)))
            vk=self.vkey(self.x,self.y)
            if vk not in self.visited or self.goal_xy:
                pass
            self.visited[vk]=time.time()
            if self.n%15==0:
                pickle.dump({'x':self.x,'y':self.y,'wallmap':self.wallmap,'trail':self.trail,'visited':list(self.visited.keys())},open('/memory/dfsstate.pkl','wb'))
                import json
                json.dump({'x':self.x,'y':self.y,'t':time.time()},open('/memory/pose.json','w'))
            # goal override
            if self.goal_xy:
                self.log(f"GOTO GOALXY {self.goal_xy}")
                self.turnto(math.degrees(math.atan2(self.goal_xy[1]-self.y,self.goal_xy[0]-self.x)),tol=4)
                res=self.goto(self.goal_xy[0],self.goal_xy[1],maxt=25)
                self.log(f"goto goal res={res}")
                if res in ('wall','bump'):
                    self.log("goal path blocked; nudge")
                    h2=r.heading()
                    self.turnto(h2+60); self.goto(self.x+0.4*math.cos(math.radians(h2+60)),self.y+0.4*math.sin(math.radians(h2+60)),maxt=4)
                continue
            ch=self.choose()
            if ch is None:
                idle+=1
                self.log(f"NO CAND (idle={idle}) spin 75")
                self.turnto(h+75)
                if idle>8:
                    self.ping(f"B2 STUCK x={self.x:.2f} y={self.y:.2f}")
                    idle=0
                continue
            idle=0
            score,k,ang,tx,ty,v,vk2,vt=ch
            self.log(f"S#{self.n} ({self.x:.2f},{self.y:.2f},{h:.0f}) -> beam{k} abs={ang%360:.0f} clear={v:.2f} new={vt is None}")
            self.turnto(ang,tol=4)
            res=self.goto(tx,ty)
            self.log(f"  res={res}")
            if res in ('wall','bump'):
                vk3=self.vkey(tx,ty)
                oct_=int(round(((ang%360)/45)))%8
                self.blocked[(vk3,oct_)]=self.blocked.get((vk3,oct_),0)+1
                self.log(f"  blocked ({vk3},{oct_}) n={self.blocked[(vk3,oct_)]}")
            time.sleep(0.05)
if __name__=="__main__":
    d=DFS()
    try: d.run()
    except Exception as ex:
        d.r.motors(0,0); d.log(f"FATAL {ex!r}")
        import traceback; traceback.print_exc(file=d.logf)
        raise
