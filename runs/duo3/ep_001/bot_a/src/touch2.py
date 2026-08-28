import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, compass
from drive import stop, motors, turn_by
from mouse import walls_here, rd6, DIRS, scan
from mouse3 import Bot3, sensors_hot
def savg(n=3):
    vs=[v for v in (rd6() for _ in range(n)) if v is not None]
    return sum(vs)/len(vs) if vs else 0
bot=Bot3(0,0)
c=compass()
while c is None: c=compass()
bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
write_port("d8","B: final contact attempt - hold still. I will nose into every opening near my best cell.")
# navigate towards higher s greedy for up to 60s (get to ~0.97 zone)
t0=time.time()
while time.time()-t0<70:
    s=savg()
    if s>0.95: break
    w,_=walls_here(bot.h)
    if w is None: continue
    bestd=None; bests=-1
    for D in DIRS:
        if w[D]: continue
        bot.face(D)
        l=scan(1)
        if l and 0<l[0]<0.33: continue
        if not bot.step(): continue
        s2=savg()
        if s2>s-0.02:
            bestd=D; break
        bot.face((D+180)%360); bot.step()
    print("pos",(bot.x,bot.y),"s",round(savg(),3),flush=True)
s=savg()
print("creep phase, s=",round(s,3),flush=True)
# at each of 16 beams: if open-ish (0.3<d) skip; creep into any obstacle 0.1-0.6m and push
L=scan(3)
order=sorted(range(16), key=lambda j: L[j] if L[j]>0 else 9)
for j in order[:6]:
    if sensors_hot(): break
    c=compass()
    turn_by(((22.5*j+180)%360)-180)
    t1=time.time()
    while time.time()-t1<4:
        if read_port("d0")=='1': break
        motors(45,45); time.sleep(0.08)
    stop()
    r=dict(d0=read_port("d0"), d7=read_port("d7"), d9=read_port("d9"), s=round(savg(),3))
    print("push beam",j,r,flush=True)
    write_port("d8", f"B pushed dir; my {r}")
    if r['d7'] not in ('0',None) or 'goal=0' not in (r['d9'] or 'goal=0'): print("!!!",flush=True); break
    motors(-70,-70); time.sleep(0.7); stop()
    cc=compass()
    bot.align(min(DIRS,key=lambda d: abs(((d-cc+540)%360)-180)))
stop()
print("done s=",round(savg(),3),flush=True)
