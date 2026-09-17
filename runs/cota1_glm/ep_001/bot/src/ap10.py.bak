import time, math, os, threading, select, re, traceback
NB=os.O_NONBLOCK
P={p:os.open(f'/dev/robot/{p}', os.O_RDONLY|NB) for p in ('d0','d1','d2','d3','d5','d6','d8')}
latest={p:'' for p in P}; pulses={'d1':0,'d6':0}
lock=threading.Lock()
def reader(p):
    buf=b''
    while True:
        try:
            r,_,_=select.select([P[p]],[],[],0.05)
            if r:
                try: d=os.read(P[p],65536)
                except BlockingIOError: continue
                if not d: continue
                buf+=d
                *lines,rest=buf.split(b'\n'); buf=rest
                with lock:
                    if lines: latest[p]=lines[-1].decode()
                    for l in lines:
                        if p in ('d1','d6') and l.strip()==b'1': pulses[p]+=1
        except Exception: pass
for p in P: threading.Thread(target=reader,args=(p,),daemon=True).start()
def get(p):
    with lock: return latest[p]
def tp():
    with lock:
        o=dict(pulses); pulses['d1']=0; pulses['d6']=0; return o
def f(x,d=0.0):
    try: return float(x)
    except: return d
def wr(p,v):
    fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|NB); os.write(fd,f'{int(v)}\n'.encode()); os.close(fd)
def emit(d4,d7,cache=[None,None,None]):
    d4=int(round(d4)); d7=int(round(d7)); t=time.time()
    try:
        if cache[0]!=d4 or t-cache[2]>1.5: wr('d4',d4); cache[0]=d4
        if cache[1]!=d7 or t-cache[2]>1.5: wr('d7',d7); cache[1]=d7
        cache[2]=t
    except Exception:
        cache[0]=cache[1]=None
LOG=open('/bot/src/auto9.log','a',buffering=1)
LAPLOG=open('/bot/src/LAPS.log','a',buffering=1)
CSV=open('/bot/src/auto9.csv','a',buffering=1)
def log(m): LOG.write(f'[{time.time()%100000:8.1f}] {m}\n')
def laplog(m): LAPLOG.write(f'[{time.time()%100000:8.1f}] {m}\n'); log('LAPBOOK '+m)
def clamp_s(v): return max(-85,min(85,v))
D8RE=re.compile(r'tick=(\d+) goal=(-?\d+) lap=(-?\d+) last=([-\d.]+) best=([-\d.]+)')
RING=[]
try:
    for l in open('/bot/src/ring.txt'):
        a=l.split(); RING.append((float(a[0]),float(a[1])))
except Exception: RING=[]
# ---- state ----
px=py=0.0; th=0.0
px=py=0.0; th=0.0
HDP=None
try:
    _p=open('/bot/src/POSE0').read().split()
    px,py=float(_p[0]),float(_p[1])
    log(f'POSE0: map-frame start ({px:.1f},{py:.1f})')
except Exception as e:
    log('POSE0 absent: '+str(e))
try:
    _h=f(get('d3'),0.0)
    th=math.radians(_h)
    log(f'th init from compass hd={_h:.1f}')
except Exception as e:
    log('FRAME INIT FAIL '+str(e))
AX,AY,ATH=7.75,-107.2,322.0       # zone (map frame)
SA=[7.75,-107.2]                 # zone point (map frame)
EXTL=[17.6,-114.2]               # departure-leg target (map frame)
JT=[10.0,-40.0]                  # junction J1 forced-turn trigger
tune={'cap':0.95,'tchk':0.0}
def load_tune():
    try:
        if time.time()-tune['tchk']>2:
            tune['tchk']=time.time()
            for kv in open('/bot/src/TUNE').read().split():
                k,v=kv.split('='); tune[k]=float(v)
            j=open('/bot/src/JTURN').read().split()
            JT[0]=float(j[0]); JT[1]=float(j[1])
    except Exception: pass
ANCH=False
sa_t=-99.0; jt_t=-99.0; forceL=False
fw_until=0.0; fw_steer=0; esc_n=0
wzbias=0.027
exit_until=uturn_until=rev_until=turn_until=rev_start=0.0
stuck_since=None; prev={'lap':None,'goal':None}; hp={'on':False,'k':0,'pause':0.0}; fired={'y':False}; osc={'init':False,'ph':'B','ax':0.0,'ay':0.0,'ux':1.0,'uy':0.0}
lastt=None; t0=time.time(); cyc=0; csvn=0; hb=0.0
log(f'v9-restart (clean) wzbias={wzbias:.4f}')
while True:
    try:
        load_tune()
        if os.path.exists('/bot/src/STOP'):
            emit(0,0); time.sleep(0.2); lastt=None; continue
        s=get('d5')
        try: b=[float(x) for x in s.split(',')]
        except Exception: time.sleep(0.03); continue
        if len(b)!=16: time.sleep(0.03); continue
        bc=[6.0 if x<0 else min(x,5.99) for x in b]
        hd=f(get('d3')); sp=f(get('d2'))
        try: wz=f(get('d0').split(',')[2])
        except Exception: wz=0
        d8=get('d8'); pl=tp(); tick='0'
        t=time.time()
        if lastt is None: lastt=t
        dt=min(t-lastt,0.2); lastt=t
        if abs(sp)<0.02 and abs(wz)<0.15: wzbias+=0.03*(wz-wzbias)
        wzc=wz-wzbias
        sp=max(-2.5,min(2.5,sp))
        if HDP is not None and abs((hd-HDP+180)%360-180)>60: hd=HDP
        HDP=hd
        thr=math.radians(hd)
        th=(th+0.25*(((thr-th+math.pi)%(2*math.pi))-math.pi))%(2*math.pi)
        if abs(wz)<0.9:
            px+=sp*math.cos(thr)*dt; py+=sp*math.sin(thr)*dt
        m=D8RE.search(d8)
        if m:
            tick,goal,lap,lastlap,best=m.groups()
            if prev['lap'] is not None and (lap!=prev['lap'] or goal!=prev['goal']):
                mx,my,mt=px,py,math.degrees(th)%360
                laplog(f'CHANGE lap {prev["lap"]}->{lap} goal {prev["goal"]}->{goal} last={lastlap} best={best} tick={tick} pose=({mx:.2f},{my:.2f}) th={mt:.1f} v={sp:.2f}')
                dx,dy=AX-mx,AY-my
                SA[0]+=dx; SA[1]+=dy; EXTL[0]+=dx; EXTL[1]+=dy; JT[0]+=dx; JT[1]+=dy
                log(f'drift-shift ({dx:.1f},{dy:.1f}) SA=({SA[0]:.1f},{SA[1]:.1f}) EXT=({EXTL[0]:.1f},{EXTL[1]:.1f})')
                if ANCH:
                    px,py,th=AX,AY,math.radians(ATH); lastt=t
                if math.hypot(mx-SA[0],my-SA[1])<12.0:
                    SA[0],SA[1]=mx,my
                    exit_until=time.time()+3.4
                    log(f'FIRE-EXIT: lap fired at start area -> EXIT toward {EXTL}')
                try: os.remove('/bot/src/HOMING')
                except Exception: pass
                try: os.remove('/bot/src/ring.txt')
                except Exception: pass
                fired['y']=True; uturn_until=0.0
                log('FIRE-CLEAR ring/uturn')
            prev['lap']=lap; prev['goal']=goal
        if pl['d1']: log(f'@@@ D1 x{pl["d1"]} pose=({px:.2f},{py:.2f}) v={sp:.2f}')
        if pl['d6']: log(f'~~~ D6 x{pl["d6"]} pose=({px:.2f},{py:.2f}) v={sp:.2f}')
        avgL=(bc[3]+bc[4]+bc[5])/3; avgR=(bc[11]+bc[12]+bc[13])/3
        FL=(bc[2]+bc[3])/2; FR=(bc[13]+bc[14])/2
        front=min(bc[15],bc[0],bc[1])
        d4=0; d7c=0; mode='?'; vt=-1
        if t>=turn_until: forceL=False
        if os.path.exists('/bot/src/OSC'):
            rearo=min(bc[8],bc[9],bc[10],bc[11],bc[12],bc[13])
            if not osc['init']:
                osc['init']=True; osc['ph']='B'; osc['ax']=px; osc['ay']=py
                osc['ux']=math.cos(th); osc['uy']=math.sin(th)
                log(f'OSC start axis hd={math.degrees(th):.0f} at ({px:.1f},{py:.1f})')
            proj=(px-osc['ax'])*osc['ux']+(py-osc['ay'])*osc['uy']
            if osc['ph']=='B':
                d4=-45; d7c=0; mode='OSC-B'
                if proj<-2.0 or rearo<0.25:
                    osc['ph']='F'; osc['ax']=px; osc['ay']=py
                    log(f'OSC->FWD at ({px:.1f},{py:.1f}) {d8}')
            else:
                d4=42; d7c=0; mode='OSC-F'
                if proj>2.0 or front<0.30:
                    osc['ph']='B'; osc['ax']=px; osc['ay']=py
                    log(f'OSC->BACK at ({px:.1f},{py:.1f}) {d8}')
            emit(d4,d7c)
        elif t<fw_until:
            d4=30 if front>0.8 else (14 if front>0.5 else 6)
            emit(d4,fw_steer); mode='FWESC'
        elif t<rev_until:
            if FL>FR+0.2: yaw=1
            elif FR>FL+0.2: yaw=-1
            else: yaw=1 if avgL>avgR else -1
            d7c=-yaw*70
            if front<0.5 and t-rev_start<4.0: rev_until=t+0.25
            emit(-45,d7c); mode='REV'
        elif t<turn_until:
            fl=None
            try: fl=open('/bot/src/FLAG').read().strip()
            except Exception: pass
            emit(22, 85 if (fl=='L' or forceL) else -85); mode='TURNIN'
        elif t<uturn_until:
            ug=157.0
            try: ug=float(open('/bot/src/UGOAL').read().strip())
            except Exception: pass
            erru=(ug-math.degrees(th)+180)%360-180
            if abs(erru)<28:
                uturn_until=0; log(f'UTURN-DONE map-hd={math.degrees(th):.0f} goal={ug:.0f}')
            elif front<0.50:
                rev_until=t+1.0; rev_start=t; emit(-45,85 if erru>0 else -85); mode='TOREV-UT'
            else:
                emit(22,85 if erru>0 else -85); mode='UTURN'
        elif t<exit_until:
            if front<0.40:
                rev_until=t+1.0; rev_start=t; exit_until=0; emit(-45,0); mode='TOREV-EXIT'
            else:
                err=math.degrees(math.atan2(EXTL[1]-py,EXTL[0]-px)-th)
                err=(err+180)%360-180
                d7c=max(-85,min(85,2.6*err))
                emit(25,d7c); mode='EXIT'
                d=math.hypot(EXTL[0]-px,EXTL[1]-py)
                if abs(err)<18 and d>2.5:
                    exit_until=0; log(f'EXIT done d={d:.1f} err={err:.0f}')
        else:
            if front<0.40:
                rev_until=t+1.0; rev_start=t; emit(-45,0); mode='TOREV'
            elif abs(sp)<0.035 and abs(wz)<0.1:
                if stuck_since is None: stuck_since=t
                if t-stuck_since>1.4:
                    rear=min(bc[8],bc[9],bc[10],bc[11],bc[12],bc[13])
                    stuck_since=None
                    if (rear<0.45 and front>0.9) or front<0.85:
                        esc_n+=1
                        if esc_n%2==1:
                            fw_steer=85 if avgL>avgR else -85
                        else:
                            fw_steer=-fw_steer
                        fw_until=t+2.2
                        log(f'STUCK-REAR n={esc_n} pose=({px:.2f},{py:.2f}) rear={rear:.2f} front={front:.2f} steer={fw_steer} -> FWD-ESCAPE')
                    else:
                        rev_until=t+1.2; rev_start=t
                        log(f'STUCK pose=({px:.2f},{py:.2f}) rear={rear:.2f} -> REV')
                else:
                    emit(0,0); mode='WAIT'
            else:
                stuck_since=None
                homing=False; veto=False
                if os.path.exists('/bot/src/ZSEEK'):
                    tgt=322.0
                    try: tgt=float(open('/bot/src/ZSEEK').read().strip())
                    except Exception: pass
                    errh=(tgt-hd+180)%360-180
                    if errh>90: errh=90
                    if errh<-90: errh=-90
                    gain=1.2*min(1.0,max(0.0,(front-0.40)/0.8))
                    d7c=max(-85,min(85,gain*errh+25*(avgL-avgR)-20*wzc))
                    d4=30 if front>0.9 else 16; mode='ZSEEK'
                    emit(d4,d7c); homing=True
                elif os.path.exists('/bot/src/HOMING') and RING:
                    if not hp['on']:
                        hp['on']=True; fired['y']=False; hp['pause']=0.0
                        gmin=(1e9,0)
                        for k,(wx,wy) in enumerate(RING):
                            dd=math.hypot(px-wx,py-wy)
                            if dd<gmin[0]: gmin=(dd,k)
                        hp['k']=gmin[1]
                        log(f'HOMING start ring idx {hp["k"]} d={gmin[0]:.1f} n={len(RING)}')
                    if hp['pause']>0 and t-hp['pause']<5.0:
                        homing=False; veto=True; mode='HPAUSE'
                    else:
                        hp['pause']=0.0
                        k0=hp['k']; lo=max(0,k0-30); hi=min(len(RING)-1,k0+30)
                        dmin=(1e9,k0)
                        for k in range(lo,hi+1):
                            wx,wy=RING[k]
                            dd=math.hypot(px-wx,py-wy)
                            if dd<dmin[0]: dmin=(dd,k)
                        if dmin[0]>20.0:
                            gmin=(1e9,0)
                            for k,(wx,wy) in enumerate(RING):
                                dd=math.hypot(px-wx,py-wy)
                                if dd<gmin[0]: gmin=(dd,k)
                            if gmin[0]<=20.0:
                                hp['k']=gmin[1]; dmin=gmin
                                log(f'HOME reacquire idx {gmin[1]} d={gmin[0]:.1f}')
                            else:
                                hp['pause']=t; hp['k']=gmin[1]
                                log(f'HOME pause d={gmin[0]:.1f} idx {gmin[1]} (HOLD)')
                                homing=False; veto=True; mode='HPAUSE'
                        if homing:
                            hp['k']=dmin[1]
                            if dmin[1]>=len(RING)-3 or math.hypot(px-RING[-1][0],py-RING[-1][1])<3.0:
                                try: os.remove('/bot/src/HOMING')
                                except Exception: pass
                                homing=False; hp['on']=False
                                if not fired['y']:
                                    uturn_until=time.time()+90
                                    log('RINGEND no fire -> UTURN goal UGOAL')
                                else:
                                    log('RINGEND after fire -> reactive')
                            else:
                                wx,wy=RING[min(len(RING)-1,dmin[1]+2)]
                                errh=math.degrees(math.atan2(wy-py,wx-px)-th)
                                errh=(errh+180)%360-180
                                steer=clamp_s(1.6*errh+20*(avgL-avgR)-18*wzc)
                                d7c=max(-85,min(85,steer))
                                d4=32 if front>0.9 else 16; mode='HOME'
                                emit(d4,d7c)
                if veto: pass
                elif not homing and math.hypot(px-JT[0],py-JT[1])<12.0 and t-jt_t>30:
                    jt_t=t; turn_until=t+2.6; forceL=True
                    log(f'JTURN pose=({px:.1f},{py:.1f}) hd={hd:.0f} -> LEFT 2.6s')
                elif math.hypot(px-SA[0],py-SA[1])<4.5 and t-sa_t>25:
                    sa_t=t; exit_until=t+3.2
                    log(f'STARTAREA pose=({px:.1f},{py:.1f}) hd={hd:.0f} -> EXIT to {EXTL}')
                else:
                    steer=85*(avgL-avgR)+45*(FL-FR)-30*wzc
                    d7c=max(-85,min(85,steer))
                    cap=tune.get('cap',0.95); vt=cap; aw=abs(wzc)
                    if aw>0.55: vt=min(vt,0.62)
                    elif aw>0.38: vt=min(vt,0.80)
                    vaf=math.sqrt(max(0.0,2*1.1*max(0.0,front-0.50)))
                    vt=min(vt,max(vaf,0.30))
                    if sp>vt+0.10: d4=0; mode='COAST'
                    elif sp>vaf+0.05 and vaf<0.9: d4=-30; mode='BRAKE'
                    else:
                        d4=38+105*(vt-0.42)+45*(vt-sp)
                        d4=max(12,min(92,d4)); mode='RUN'
                    emit(d4,d7c)
        if mode in ('?','TOREV','TOREV-UT','TOREV-EXIT','WAIT','HPAUSE'): emit(d4,d7c)
        csvn+=1
        if csvn%4==0:
            CSV.write(f'{t-t0:.2f},{tick},{px:.3f},{py:.3f},{math.degrees(th)%360:.1f},{hd:.1f},{sp:.3f},{wz:.3f},{d4},{d7c},{mode},{front:.2f},{avgL:.2f},{avgR:.2f},{vt:.2f}\n')
        cyc+=1
        if cyc%400==0: log(f'c{cyc} hd={hd:.0f} v={sp:.2f} pose=({px:.1f},{py:.1f}) front={front:.2f} {mode} hm={hp["on"]}/{hp["k"]}/{"Y" if os.path.exists("/bot/src/HOMING") else "N"} {d8}')
        if cyc%100==0: log(f'H-DIAG mode={mode} homing={homing} file={"Y" if os.path.exists("/bot/src/HOMING") else "N"} k={hp["k"]} pose=({px:.1f},{py:.1f}) hd={hd:.0f} th={math.degrees(th):.0f} sp={sp:.2f}')
        if t-hb>2:
            hb=t
            with open('/bot/src/HB9','w') as h: h.write(f'{t:.1f} cyc={cyc} pose=({px:.1f},{py:.1f}) v={sp:.2f} {mode}\n')
        time.sleep(0.045)
    except Exception:
        try: log('EXC '+traceback.format_exc().replace('\n',' | ')[-300:])
        except Exception: pass
        try: emit(0,0)
        except Exception: pass
        time.sleep(0.05)
