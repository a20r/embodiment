import sys, time, math, json
from collections import deque
sys.path.insert(0,'/bot/src')
from robot import R
GOALS=[(0.94,-1.15),(2.00,-0.62)]
GI=0
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R(); r.motors(0,0); time.sleep(0.3)
wallmap={}
def pose():
    try:
        j=json.load(open('/memory/pose.json'))
        return j['x'],j['y']
    except: return None,None
def stamp(x,y,h,rg):
    for k,val in enumerate(rg):
        if val is None or val<0 or val>1.9: continue
        a=math.radians(h+k*22.5)
        wx=x+val*math.cos(a); wy=y+val*math.sin(a)
        key=(int(round(wx/0.1)),int(round(wy/0.1)))
        wallmap[key]=min(wallmap.get(key,0)+1,6)
def turnto(t,tol=4.0):
    for _ in range(60):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(t-h)
        if abs(e)<tol: r.motors(0,0); time.sleep(0.05); return
        v=int(max(-80,min(80,2.2*e)))
        r.motors(v,-v); time.sleep(0.07)
    r.motors(0,0)
def goto(tx,ty,maxt=5.0,speed=85):
    t0=time.time()
    while time.time()-t0<maxt:
        st=r.status() or (0,0,0)
        if st[2]==1: r.motors(0,0); return 'HERE'
        px,py=pose()
        if px is None: r.motors(0,0); time.sleep(0.05); continue
        if math.hypot(tx-px,ty-py)<0.16:
            r.motors(0,0); return 'ok'
        rg=r.ranges()
        f0=rg[0] if (rg and rg[0] is not None and rg[0]>=0) else 9
        if f0<0.27:
            r.motors(0,0); return 'wall'
        b=math.degrees(math.atan2(ty-py,tx-px))
        err=wrap(b-(r.heading() or b))
        a=int(max(-40,min(100, speed+1.6*err)))
        r.motors(a,a); time.sleep(0.09)
    r.motors(0,0); return 'to'
def bfs(start,goalc):
    G={}
    for (wi,wj),c in wallmap.items():
        if c>=2:
            G[(wi,wj)]=1
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                G[(wi+di,wj+dj)]=1
    seen={start:None}; q=deque([start]); N=0
    while q:
        cur=q.popleft(); N+=1
        if N>20000: break
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
t0=time.time(); side=1; lastdist=None; lastprog=time.time()
print("RUSH4 start",flush=True)
while time.time()-t0<2400:
    st=r.status() or (0,0,0)
    if st[2]==1:
        print("*** HERE=1 ON GOAL ***",flush=True)
        r.motors(0,0)
        while True:
            r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
            time.sleep(1.0)
    r.motors(0,0); time.sleep(0.2)
    px,py=pose()
    if px is None: time.sleep(0.2); continue
    h=r.heading()
    rg=r.ranges()
    if rg is not None and h is not None:
        stamp(px,py,h,rg)
    dist=math.hypot(GOALS[GI][0]-px,GOALS[GI][1]-py)
    bear=math.degrees(math.atan2(GOALS[GI][1]-py,GOALS[GI][0]-px))
    print(f"t={time.time()-t0:.0f} d={dist:.2f} b={bear%360:.0f} pose=({px:.2f},{py:.2f}) walls={len(wallmap)}",flush=True)
    if dist<1.3:
        print(f"CLOSE to candidate {GI} - precise search",flush=True)
        found=False
        yy=-1.2
        while yy<=1.2 and not found:
            xs=[-1.2+i*0.25 for i in range(11)]
            if int(round(yy/0.25))%2: xs=xs[::-1]
            for xx in xs:
                st=r.status() or (0,0,0)
                if st[2]==1: found=True; break
                txp=GOALS[GI][0]+xx; typ=GOALS[GI][1]+yy
                res=goto(txp,typ,maxt=3.0,speed=45)
                if res=='HERE': found=True; break
                r.motors(0,0); time.sleep(0.15)
            if found: break
            yy+=0.25
        if found:
            print("*** HERE=1 ON GOAL ***",flush=True)
            r.motors(0,0)
            while True:
                r.write(8,f"B2 ATGOAL ON GOAL t={time.time():.0f}")
                time.sleep(1.0)
        else:
            print(f"candidate {GI} exhausted - switching",flush=True)
            GI=(GI+1)%2
        continue
    start=(int(round(px/0.1)),int(round(py/0.1)))
    goalc=(int(round(GOALS[GI][0]/0.1)),int(round(GOALS[GI][1]/0.1)))
    path=bfs(start,goalc)
    moved=False
    if path and len(path)>3:
        # execute up to 2.2m of path in 0.45m chunks
        tx,ty=px,py; acc=0
        for (ci,cj) in path[::5]:
            wx,wy=ci*0.1,cj*0.1
            if math.hypot(wx-px,wy-py)>2.2: break
            tx,ty=wx,wy
        res=goto(tx,ty,maxt=9)
        moved = res in ('ok','to')
        print(f"  BFS path={len(path)} -> ({tx:.2f},{ty:.2f}) res={res}",flush=True)
        if res=='wall':
            bx=px+0.3*math.cos(math.radians(r.heading() or bear)); by=py+0.3*math.sin(math.radians(r.heading() or bear))
            bk=(int(round(bx/0.1)),int(round(by/0.1)))
            wallmap[bk]=wallmap.get(bk,0)+3
            r.motors(-60,-60); time.sleep(0.4); r.motors(0,0)
    else:
        # greedy beam with goal bias, else wall-follow
        rg2=r.ranges(); h2=r.heading()
        if rg2 and h2 is not None:
            best=None; bestcost=1e9
            for k in range(16):
                v=rg2[k]
                if v is None or v<0 or v<0.5: continue
                angk=h2+k*22.5
                dd=abs(wrap(angk-bear))
                cost=dd - min(v,1.5)*20
                if cost<bestcost: bestcost=cost; best=(k,angk,v)
            if best is not None and bestcost<150:
                k,angk,v=best
                turnto(angk)
                res=goto(px+0.35*math.cos(math.radians(angk)),py+0.35*math.sin(math.radians(angk)),maxt=2.5)
                print(f"  GREEDY beam{k} d={wrap(angk-bear):.0f} v={v:.2f} res={res}",flush=True)
                if res=='wall':
                    bx=px+0.3*math.cos(math.radians(angk)); by=py+0.3*math.sin(math.radians(angk))
                    bk=(int(round(bx/0.1)),int(round(by/0.1)))
                    wallmap[bk]=wallmap.get(bk,0)+3
            else:
                # left wall follow 1.3s
                t1=time.time(); rg3=r.ranges()
                while time.time()-t1<1.3:
                    st2=r.status() or (0,0,0)
                    if st2[2]==1: break
                    rg3=r.ranges()
                    if rg3 is None: r.motors(0,0); time.sleep(0.05); continue
                    f0=rg3[0] if (rg3[0] is not None and rg3[0]>=0) else 9
                    if f0<0.42:
                        r.motors(-65,65); time.sleep(0.12); continue
                    L=[rg3[i] for i in (3,4,5) if rg3[i] is not None and rg3[i]>=0]
                    dl=min([x*1.3 for x in L]) if L else 0.5
                    err=dl-0.30
                    a=int(max(-30,min(100, 55+130*err)))
                    b2=int(max(-30,min(100, 55-130*err)))
                    r.motors(a,b2); time.sleep(0.09)
                r.motors(0,0)
                print("  WALLFOLLOW 1.3s",flush=True)
    if lastdist is not None and lastdist-dist>0.2: lastprog=time.time()
    lastdist=dist
    if time.time()-lastprog>20:
        print("STUCK LONG - breakout",flush=True)
        turnto(bear+120*side)
        goto(px+0.7*math.cos(math.radians(bear+120*side)),py+0.7*math.sin(math.radians(bear+120*side)),maxt=3)
        side*=-1; lastprog=time.time()
