import sys; sys.path.insert(0,'/bot/src')
import ctl, rio, time, math, json
TPU=1850.0
PF='/bot/pose.json'
def log(m):
    with open('/bot/explore.log','a') as f: f.write(f"{time.strftime('%H:%M:%S')} SEEK {m}\n")
def d11(n=4):
    v=[]
    for _ in range(n):
        s=rio.read_line('d11',0.5)
        try: v.append(float(s))
        except: pass
        time.sleep(0.08)
    return sum(v)/len(v) if v else 0
def dir_vec(h): r=math.radians(h); return math.sin(r),math.cos(r)
st=json.load(open(PF)); x,y=st['x'],st['y']
bad={}  # cell -> set of bad headings
def cellkey(): return f"{int(math.floor(x/0.3))},{int(math.floor(y/0.3))}"
base=d11(); log(f'start d11={base:.3f} pose=({x:.2f},{y:.2f})')
last_dir=None
t_end=time.time()+float(sys.argv[1]) if len(sys.argv)>1 else time.time()+300
while time.time()<t_end:
    if base>0.95: log('very close, stop'); break
    s=ctl.scan(); h=ctl.heading()
    if not s or h is None: continue
    cands=[]
    for k in range(16):
        r=s[k]
        if r<0.4: continue
        rl=s[(k-1)%16]; rr=s[(k+1)%16]
        if (0<=rl<0.15) or (0<=rr<0.15): continue
        ang=(h+22.5*k)%360
        if any(abs(ctl.angdiff(ang,b))<30 for b in bad.get(cellkey(),[])): continue
        sc=min(r,1.0)
        if last_dir is not None: sc+=0.5*math.cos(math.radians(ctl.angdiff(ang,last_dir)))  # prefer continuing
        cands.append((sc,ang,r))
    if not cands:
        log('no candidates; clearing bad + backing'); bad[cellkey()]=[]; l0,r0=ctl.enc(); ctl.drive(-40,-40); time.sleep(0.8); ctl.stop(); l1,r1=ctl.enc(); dd=((l1-l0)+(r1-r0))/2/TPU; dx,dy=dir_vec(h); x+=dx*dd; y+=dy*dd; continue
    cands.sort(reverse=True); _,ang,r=cands[0]
    ck=cellkey()
    if abs(ctl.angdiff(ang,h))>6: ctl.turn_to(ang,tol=4)
    l0,r0=ctl.enc(); trav,reason=ctl.forward(min(r-0.2,0.4)*TPU,speed=60,hold_heading=ang,min_front=0.18,timeout=8)
    l1,r1=ctl.enc(); dd=((l1-l0)+(r1-r0))/2/TPU; dx,dy=dir_vec(ang); x+=dx*dd; y+=dy*dd
    time.sleep(0.3); now=d11()
    stt=ctl.status()
    log(f'dir={ang:.0f} moved={trav:.0f} {reason} d11 {base:.3f}->{now:.3f} pose=({x:.2f},{y:.2f}) here={stt.get("here")}')
    json.dump({**st,'x':x,'y':y},open(PF,'w'))
    if now<base-0.02:
        bad.setdefault(ck,[]).append(ang)
        # go back
        back=(ang+180)%360; ctl.turn_to(back,tol=4); l0,r0=ctl.enc(); trav2,_=ctl.forward(abs(trav),speed=60,hold_heading=back,min_front=0.18,timeout=8)
        l1,r1=ctl.enc(); dd=((l1-l0)+(r1-r0))/2/TPU; dx,dy=dir_vec(back); x+=dx*dd; y+=dy*dd
        last_dir=None; log(f'worse; backtracked {trav2:.0f}')
        base=max(base, d11())
    else:
        base=now; last_dir=ang
ctl.stop(); json.dump({**st,'x':x,'y':y},open(PF,'w')); log(f'end d11={base:.3f} pose=({x:.2f},{y:.2f})')
