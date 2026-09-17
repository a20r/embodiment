import ctl, rio, time, math, json, sys
open('/bot/src/explore2.pid','w').write(str(__import__('os').getpid()))
LOG=open('/bot/src/explore2.log','a')
S=1700.0; CELL=0.6
x=float(sys.argv[2]) if len(sys.argv)>2 else 0.0; y=float(sys.argv[3]) if len(sys.argv)>3 else 0.0; L0,R0=ctl.enc()
def odo():
    global x,y,L0,R0
    L,R=ctl.enc(); d=((L-L0)+(R-R0))/2/S; L0,R0=L,R
    h=math.radians(ctl.heading(1)); x+=d*math.cos(h); y+=d*math.sin(h)
def log(ev, sc=None, extra=''):
    rec={'t':round(time.time(),1),'x':round(x,2),'y':round(y,2),'h':round(ctl.heading(1)),'ev':ev,'d11':rio.rd('d11'),'d3':rio.rd('d3')}
    if sc: rec['sc']=[round(v,2) for v in sc]
    if extra: rec['x_']=extra
    m=rio.rd('d10',0.05)
    if m: rec['rx']=m
    LOG.write(json.dumps(rec)+'\n'); LOG.flush(); return rec
def align():
    h=ctl.heading(5); tgt=round(h/90)*90
    if abs(ctl.angdiff(tgt,h))>4: ctl.turn(ctl.angdiff(tgt,h), v=15)
def drive(dist, v=60):
    """drive dist lidar units, centering between walls; returns (encdist, front_before, front_after, reason)"""
    L0_,R0_=ctl.enc(); h0=round(ctl.heading(5)/90)*90; sc0=ctl.scan(2); f0=ctl.front(sc0); reason='dist'; t0=time.time()
    while True:
        L,R=ctl.enc(); d=((L-L0_)+(R-R0_))/2/S
        if d>=dist: break
        if time.time()-t0>dist*S/(5*v)+6: reason='timeout'; break
        sc=ctl.scan(1); f=ctl.front(sc)
        if 0<=f<0.27: reason='front'; break
        l=sc[4]; r=sc[12]
        lat=0.0
        if 0<l<0.7 and 0<r<0.7: lat=(l-r)          # >0 => more room left => steer left(CCW)
        elif 0<r<0.22: lat=(0.22-r)*2
        elif 0<l<0.22: lat=-(0.22-l)*2
        herr=ctl.angdiff(h0,ctl.heading(1))          # >0 => need CCW
        corr=max(-0.4,min(0.4, 0.015*herr + 1.2*lat))
        rio.wr('d7',str(v*(1-corr))); rio.wr('d1',str(v*(1+corr)*1.05))
        time.sleep(0.08)
    ctl.stop(); time.sleep(0.2)
    L,R=ctl.enc(); sc1=ctl.scan(2)
    return ((L-L0_)+(R-R0_))/2, f0, ctl.front(sc1), reason
lasttx=0; steps=0; visited={}; homing=False; sig={}
log('start')
while True:
    odo(); align(); odo()
    sc=ctl.scan(3); rec=log('scan',sc)
    if 'goal=0' not in (rec['d3'] or 'goal=0'):
        ctl.stop(); log('GOAL'); open('/bot/src/tx_msg.txt','w').write('GOAL AT (%.1f,%.1f) in my frame. A is ON THE GOAL, stopped, waiting for you. Home in on rising d11 (signal). Tell me when you arrive.\n'%(x,y))
        while True:
            time.sleep(5); r=log('ONGOAL'); 
            if 'goal=0' in (r['d3'] or ''): break
        continue
    if rec.get('rx') and 'GOAL AT' in rec['rx'].upper() and not homing:
        homing=True; log('HOMING-ON',None,rec['rx'])
    try: sig[(round(x/CELL),round(y/CELL))]=float(rec['d11'])
    except: pass
    if time.time()-lasttx>20:
        rio.wr('d8','Robot A exploring. odo x=%.1f y=%.1f h=%d. Where are you? Found goal?'%(x,y,ctl.heading(1))); lasttx=time.time()
    f=ctl.front(sc); l=sc[4] if sc[4]>=0 else 9; r=sc[12] if sc[12]>=0 else 9
    if 0<=min(sc[3],sc[5])<0.25: l=min(l,0.5)
    if 0<=min(sc[11],sc[13])<0.25: r=min(r,0.5)
    b=sc[8] if sc[8]>=0 else 9
    # right-hand rule: right if open, else straight, else left, else back
    BB=(-3.0,1.0,-0.5,3.4)  # B explored bbox
    h=round(ctl.heading(3)/90)*90
    opts=[]
    for name,ang,dist in (('L',90,l),('F',0,f),('R',-90,r),('B',180,b)):
        if dist<0.6: continue
        a=math.radians(h+ang); nx,ny=x+CELL*math.cos(a),y+CELL*math.sin(a)
        cell=(round(nx/CELL),round(ny/CELL))
        score=0.0
        if not (BB[0]<=nx<=BB[1] and BB[2]<=ny<=BB[3]): score+=3
        if cell not in visited: score+=2
        else: score-=visited[cell]
        if name=='B': score-=1.5
        if name=='F': score+=0.3
        score+=min(dist,2.5)*0.3
        if homing:
            here=sig.get((round(x/CELL),round(y/CELL)),0.5); there=sig.get(cell)
            score = (there-here)*30 if there is not None else 2.0+(0.5 if name=='F' else 0)
            if name=='B': score-=1.0
        opts.append((score,name,ang))
    if not opts: opts=[(0,'B',180)]
    opts.sort(reverse=True); sc_,name,ang=opts[0]
    if ang: ctl.turn(ang)
    log('turn'+name,None,'opts=%s'%[(round(s,1),n) for s,n,a in opts])
    visited[(round(x/CELL),round(y/CELL))]=visited.get((round(x/CELL),round(y/CELL)),0)+1
    odo()
    d,f0,f1,reason=drive(CELL, v=60); odo(); steps+=1
    exp=d/S; got=f0-f1
    slip = (f0<2.2 and f0>=0 and f1>=0 and got<0.35*exp and exp>0.2)
    log('drive',None,'d=%d exp=%.2f got=%.2f %s%s'%(d,exp,got,reason,' SLIP' if slip else ''))
    if slip:
        # back off and nudge toward center
        rio.wr('d7','-40'); rio.wr('d1','-42'); time.sleep(0.8); ctl.stop(); odo()
        sc=ctl.scan(2)
        if 0<=sc[12]<sc[4] or sc[4]<0: ctl.turn(35); ctl.forward(0.12*S, v=40, stopfront=0.2); ctl.turn(-35)
        else: ctl.turn(-35); ctl.forward(0.12*S, v=40, stopfront=0.2); ctl.turn(35)
        odo(); log('unstick')
