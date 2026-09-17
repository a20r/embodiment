#!/usr/bin/env python3
import os,time,math,json,subprocess
R='/dev/robot/'
def wr(p,s):
    try:
        fd=os.open(R+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,(s+'\n').encode()); os.close(fd)
    except Exception: pass
def rd(p,to=0.15):
    try:
        r=subprocess.run(['timeout',str(to),'cat',R+p],capture_output=True,text=True)
        return r.stdout.strip()
    except Exception: return ''
def motors(l,r_): wr('d7',str(int(l))); wr('d1',str(int(r_)))
def stop(): motors(0,0)
LOG=open('/memory/ep2.log','a',buffering=1)
def log(**kw): LOG.write(json.dumps(kw)+'\n')
def parse_d3(s):
    d={}
    for tok in s.split():
        if '=' in tok:
            k,v=tok.split('='); d[k]=v
    return d
# pose init from encoders
e_l=float(rd('d6','0.4') or 0); e_r=float(rd('d9','0.4') or 0)
x=y=0.0
H=float(rd('d4','0.4') or 0)
c=0; t0=time.time(); last_tx=0; d0_hits=[]; hold=0; unstick_t=time.time(); unstick_x=x; unstick_y=y
bias=0.0; bias_t=0
stop()
time.sleep(0.2)
while time.time()-t0<150:
    c+=1
    now=time.time()
    # sensors
    d0v=rd('d0','0.05'); d5v=rd('d5','0.05')
    fl=rd('d3','0.15'); fl_d=parse_d3(fl)
    raw=rd('d2','0.4')
    h4=rd('d4','0.1')
    el=rd('d6','0.1'); er=rd('d9','0.1')
    try: H=float(h4)
    except Exception: pass
    # lidar front clearance
    fc=9.9; lmin=9.9; rmin=9.9; fmin=9.9
    for tok in raw.split(';'):
        try: xx,yy,zz=map(float,tok.split(','))[:3]
        except Exception:
            sp=tok.split(',')
            if len(sp)<2: continue
            try: xx=float(sp[0]); yy=float(sp[1])
            except Exception: continue
        r=math.hypot(xx,yy)
        if r<0.20: continue
        az=math.degrees(math.atan2(yy,xx))
        if r<0.55 and not(-50<=az<=35): continue
        if -22<=az<=22 and r<fc: fc=r
        if -50<=az<=-15 and r<lmin: lmin=r
        if 10<=az<=35 and r<rmin: rmin=r
        if 0.2<r<1.2: fmin=min(fmin,r)  # any mid obstacle
    # odometry update
    try:
        nel=float(el); ner=float(er)
        dC=((nel-e_l)+(ner-e_r))/2.0
        if abs(dC)<200:
            x+=dC*math.cos(math.radians(H))/1000.0
            y+=dC*math.sin(math.radians(H))/1000.0
        e_l,e_r=nel,ner
    except Exception: pass
    # radio
    if now-last_tx>3.0:
        wr('d8','A x=%.1f y=%.1f H=%.0f'%(x,y,H)); last_tx=now
    if c%5==0:
        rx=rd('d10','0.15')
        if rx: log(ev='RX',msg=rx); print('RX:',rx,flush=True)
    # d0 handling
    if d0v=='1':
        d0_hits.append(now)
        d0_hits=[t for t in d0_hits if now-t<2.0]
        if len(d0_hits)>=2 and hold==0:
            hold=now; stop()
            log(ev='D0HIT',x=x,y=y,H=H,fc=fc)
            print('D0 HIT at %.2f %.2f'%(x,y),flush=True)
    if hold:
        if now-hold<10:
            motors(0,0)
            wr('d8','A D0HERE x=%.1f y=%.1f'%(x,y))
            time.sleep(0.1); continue
        else:
            # evaluate: if d0 flickered during hold, stay 40 more sec pulsing
            dr=sum(1 for t in d0_hits if now-t<12)
            if dr>=3 and now-hold<50:
                motors(0,0); time.sleep(0.1); continue
            hold=0; d0_hits=[]
    # stuck detection
    if now-unstick_t>7:
        if math.hypot(x-unstick_x,y-unstick_y)<0.05:
            log(ev='UNSTICK'); spin_dir=1 if (c%2) else -1
            for _ in range(8): motors(20*spin_dir,-20*spin_dir); time.sleep(0.1)
            stop(); time.sleep(0.2)
        unstick_t=now; unstick_x=x; unstick_y=y
    # wandering bias
    if now-bias_t>6:
        bias_t=now; bias=(now*7919%13-6)*1.5
    # control: steer toward most-open bin
    bins={}
    for tok in raw.split(';'):
        try: xx,yy,zz=map(float,tok.split(','))[:3]
        except Exception: continue
        rr=math.hypot(xx,yy)
        if rr<0.20: continue
        az=math.degrees(math.atan2(yy,xx))
        if az<-55 or az>40: continue
        k=round(az/10)*10
        v=bins.get(k)
        if v is None or rr<v[0]: bins[k]=(rr,1)
        else: bins[k]=(v[0],v[1]+1)
    # score bins: min clearance, require >=3 pts
    cand=[(k,v[0]) for k,v in bins.items() if v[1]>=3]
    tgt=0.0; best=-1
    for k,v in cand:
        sc=v - abs(k)*0.004
        if sc>best: best=sc; tgt=k
    if fc<0.30 or best>9.0:
        err=tgt*1.1
    else:
        err=(rmin-lmin)*4.0+tgt*0.3+bias
    base=25
    err=max(-25,min(25,err))
    L=base-int(err); Rc=base+int(err)
    L,Rc=max(0,L),max(0,Rc)
    motors(L,Rc)
    if c%20==0:
        log(c=c,x=x,y=y,H=H,fc=fc,l=lmin,r=rmin,d0=d0v,d5=d5v,goal=fl_d.get('goal'),here=fl_d.get('here'),tick=fl_d.get('tick'))
    time.sleep(0.08)
stop()
log(ev='END')
print('explore done',flush=True)
