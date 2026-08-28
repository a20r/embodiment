import lib, time, math, random, collections

LOG = open("/memory/trail.csv","a")
prev=[0.5]*16
def clean(l):
    global prev
    out=[(prev[i] if v<0 else v) for i,v in enumerate(l)]
    prev=out; return out

def enc():
    try: return int(lib.read("d7")), int(lib.read("d8"))
    except: return None

x,y=0.0,0.0
e_last=enc()
CPU=700.0
def odom():
    global x,y,e_last
    e=enc()
    if e and e_last:
        d=((e[0]-e_last[0])+(e[1]-e_last[1]))/2.0/CPU
        h=math.radians(lib.heading())
        x+=d*math.cos(h); y+=d*math.sin(h)
        e_last=e
    return x,y

visits=collections.Counter()
side = 1   # 1 = follow right wall, -1 = follow left wall
t0=time.time()
last_e=e_last; last_move_t=time.time()
while time.time()-t0<3000:
    l=clean(lib.lidar())
    g,gs=lib.goal()
    radio=lib.read("d5")
    ox,oy=odom(); h=lib.heading()
    LOG.write(f"{time.time():.1f},{ox:.2f},{oy:.2f},{h:.1f},{','.join(f'{v:.2f}' for v in l)},{gs},,,{radio}\n"); LOG.flush()
    if g:
        lib.stop(); print("GOAL!",gs,flush=True); break
    if radio.strip(): print("RADIO:",radio,flush=True)
    cell=(round(ox/0.4),round(oy/0.4))
    visits[cell]+=1
    if visits[cell]>25:
        side=-side; visits.clear()
        print(f"loop detected at {cell}, switching side to {side}",flush=True)
        # escape: turn to most open dir and drive a bit
        j=max(range(16),key=lambda i:l[i])
        lib.turn_by(((22.5*j+180)%360)-180)
        lib.wheels(40,40); time.sleep(1.0)
        continue
    # stuck detection
    e=enc()
    if e:
        if abs(e[0]-last_e[0])+abs(e[1]-last_e[1])>15:
            last_move_t=time.time(); last_e=e
        elif time.time()-last_move_t>4:
            print("stuck, backing",flush=True)
            lib.wheels(-30,-30); time.sleep(1.2)
            lib.wheels(20*side,-20*side); time.sleep(1.0)
            lib.stop(); last_move_t=time.time(); last_e=enc()
            continue
    front=min(l[0], l[1]*1.3, l[15]*1.3)
    if side==1:
        wall=min(l[12], l[13]*1.15, l[11]*1.15)
    else:
        wall=min(l[4], l[3]*1.15, l[5]*1.15)
    if front<0.35:
        lib.wheels(-14*side,14*side); time.sleep(0.3); continue
    err=wall-0.35
    err=max(-0.3,min(0.3,err))
    turn=err*45*side
    base=40 if front>1.0 else (22 if front>0.6 else 14)
    lib.wheels(round(base+turn,1),round(base-turn,1))
    time.sleep(0.22)
lib.stop(); print("done",flush=True)
