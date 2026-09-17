import sys; sys.path.insert(0,'/bot/src')
import ctl, rio, time, math, json
TPU=1850.0; PF='/bot/pose.json'
hdg_travel=float(sys.argv[1]); side=sys.argv[2]  # 'right' or 'left' wall to follow
maxdist=float(sys.argv[3]) if len(sys.argv)>3 else 2.0
st=json.load(open(PF)); x,y=st['x'],st['y']
def dv(h): r=math.radians(h); return math.sin(r),math.cos(r)
ctl.turn_to(hdg_travel,tol=4)
l0,r0=ctl.enc(); t0=time.time(); res='timeout'
while time.time()-t0<50:
    s=ctl.scan(); h=ctl.heading()
    if not s or h is None: continue
    l,r=ctl.enc(); trav=((l-l0)+(r-r0))/2/TPU
    if trav>maxdist: res='maxdist'; break
    front=min(v for v in (s[0],s[1],s[15]) if v>0) if any(v>0 for v in (s[0],s[1],s[15])) else 9
    if front<0.22: res='front_blocked'; break
    sd = s[4] if side=='right' else s[12]
    sd2 = s[3] if side=='right' else s[13]
    if sd>0.55 and sd2>0.45 and trav>0.15: res='opening'; break
    # keep side distance ~0.25, hold heading
    e_h=ctl.angdiff(hdg_travel,h)
    e_s=(0.25-sd) if sd>0 else 0
    corr=max(-12,min(12,e_h*0.8 + (e_s*40 if side=='right' else -e_s*40)))
    ctl.drive(45+corr,45-corr); time.sleep(0.05)
ctl.stop(); l,r=ctl.enc(); trav=((l-l0)+(r-r0))/2/TPU; dx,dy=dv(hdg_travel); x+=dx*trav; y+=dy*trav
json.dump({**st,'x':x,'y':y},open(PF,'w'))
h=ctl.heading(); s=ctl.scan()
print(res, f'trav={trav:.2f} pose=({x:.2f},{y:.2f}) hdg={h:.0f} d11={rio.read_line("d11",1)} {ctl.status()}')
print(' '.join(f'{(h+22.5*k)%360:3.0f}:{s[k]:.2f}' for k in range(16)))
