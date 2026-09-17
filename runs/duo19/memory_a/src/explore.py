import ctl, rio, time, math, json, sys, statistics
LOG=open('/bot/src/explore.log','a')
x=y=0.0; L0,R0=ctl.enc()
S=2200.0  # enc units per lidar unit
def odo():
    global x,y,L0,R0
    L,R=ctl.enc(); d=((L-L0)+(R-R0))/2/S; L0,R0=L,R
    h=math.radians(ctl.heading(1)); x+=d*math.cos(h); y+=d*math.sin(h)
def log(ev, sc=None):
    rec={'t':round(time.time(),1),'x':round(x,2),'y':round(y,2),'h':round(ctl.heading(1)),'ev':ev,'d11':rio.rd('d11'),'d3':rio.rd('d3')}
    if sc: rec['sc']=[round(v,2) for v in sc]
    m=rio.rd('d10',0.05)
    if m: rec['rx']=m
    LOG.write(json.dumps(rec)+'\n'); LOG.flush()
    return rec
def goal_reached(rec):
    return 'goal=0' not in (rec.get('d3') or 'goal=0')
V=float(sys.argv[1]) if len(sys.argv)>1 else 60
TARGET=0.28
lasttx=0; lastsc=None; still=0
log('start')
while True:
    odo()
    sc=ctl.scan(2)
    rec=log('run',sc)
    if goal_reached(rec):
        ctl.stop(); log('GOAL'); print('GOAL'); break
    if time.time()-lasttx>15:
        rio.wr('d8','Robot A here, exploring maze by right-wall-following. odo x=%.1f y=%.1f h=%d. Where are you? Have you found the goal?'%(x,y,ctl.heading(1))); lasttx=time.time()
    f=ctl.front(sc); right=sc[12] if sc[12]>0 else 9; fr=sc[14] if sc[14]>0 else 9; left=sc[4] if sc[4]>0 else 9
    # stuck detection: scan unchanged while driving
    if lastsc and max(abs(a-b) for a,b in zip(sc,lastsc))<0.03: still+=1
    else: still=0
    lastsc=sc
    if still>=8:
        log('stuck-backup'); ctl.stop(); rio.wr('d7','-40'); rio.wr('d1','-40'); time.sleep(1.5); ctl.stop(); ctl.turn(45); still=0; continue
    if f<0.32:
        ctl.stop(); odo()
        # choose: left open? else back
        sc=ctl.scan(3); log('blocked',sc)
        if sc[4]>0.5 or sc[4]<0: ctl.turn(90); log('turnL')
        elif sc[12]>0.5: ctl.turn(-90); log('turnR')
        else: ctl.turn(180); log('turnBack')
        continue
    if right>0.75 and fr>0.6:
        # opening on right: move ahead to center of the opening then turn right
        ctl.forward(0.45*S, v=V, stopfront=0.3); odo(); log('rightopen',ctl.scan(2))
        ctl.turn(-90); odo(); log('turnR-open')
        sc=ctl.scan(2)
        if ctl.front(sc)<0.35: ctl.turn(90); log('turnR-abort')
        continue
    # wall follow: steer to keep right distance ~TARGET
    err=TARGET-right if right<1.5 else 0.0   # err>0 => too close to right => steer left (CCW) => right wheel faster
    corr=max(-0.35,min(0.35,err*2.0))
    rio.wr('d7',str(V*(1-corr))); rio.wr('d1',str(V*(1+corr)*1.05))
    time.sleep(0.15)
