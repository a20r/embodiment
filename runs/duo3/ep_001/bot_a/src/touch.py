import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, compass
from drive import stop, motors
from mouse import walls_here, rd6, DIRS, scan
from mouse3 import Bot3, sensors_hot
def savg(n=3):
    vs=[v for v in (rd6() for _ in range(n)) if v is not None]
    return sum(vs)/len(vs) if vs else 0
bot=Bot3(0,0)
c=compass()
while c is None: c=compass()
bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
for _ in range(4):
    write_port("d8","B: HOLD EXACTLY STILL 3 minutes. I will sweep nearby cells and try to reach/touch you. Watch your d7/d9.")
M={}; S={}; fails={}
t_end=time.time()+170
best=(0,None)
while time.time()<t_end:
    if sensors_hot(): print("HOT"); break
    key=(bot.x,bot.y)
    s=savg(); S[key]=s
    if s>best[0]: best=(s,key)
    if key not in M:
        w,_=walls_here(bot.h)
        if w is None: continue
        M[key]={k:v for k,v in w.items()}
    print(key,"s",round(s,3),flush=True)
    # contact attempt: if s>0.93, creep toward nearest obstacle beam & bump it gently
    if s>0.985:
        L=scan(2)
        j=min(range(16), key=lambda i: L[i] if L[i]>0 else 9)
        # face that direction approx: absolute dir = h + 22.5*j -> pick nearest cardinal? creep rotate precisely:
        from drive import turn_by
        turn_by(22.5*j if 22.5*j<=180 else 22.5*j-360)
        t0=time.time()
        while time.time()-t0<3:
            if read_port("d0")=='1': break
            l=scan(1)
            if l and 0<l[0]<0.1: break
            motors(50,50); time.sleep(0.1)
        stop()
        print("bump attempt done: d0",read_port("d0"),"d7",read_port("d7"),"d9",read_port("d9"),"s",round(savg(),3),flush=True)
        write_port("d8", f"B bumped obstacle; my d7={read_port('d7')} d9={read_port('d9')}. If your d7 flipped say so!")
        motors(-60,-60); time.sleep(0.8); stop()
        c=compass()
        bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
        time.sleep(2)
        continue
    # move: prefer unvisited open neighbor with... just DFS-ish greedy by unknown, else revisit toward best
    w=M[key]
    opts=[]
    for D in DIRS:
        if not w[D] and fails.get((key,D),0)<2:
            v=(bot.x+DIRS[D][0], bot.y+DIRS[D][1])
            sc = 1.0 if v not in S else S[v]*0.8
            opts.append((sc,D,v))
    if not opts:
        print("stuck",flush=True); break
    opts.sort(reverse=True)
    _,D,v=opts[0]
    bot.face(D)
    l=scan(2)
    if l and 0<l[0]<0.33:
        M[key][D]=True; continue
    if not bot.step(): fails[(key,D)]=fails.get((key,D),0)+1
stop()
print("best",best,flush=True)
