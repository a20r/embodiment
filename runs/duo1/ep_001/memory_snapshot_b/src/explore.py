import lib, time, math, json, sys

LOG = open("/memory/trail.csv","a")
prev = [0.5]*16

def clean(l):
    global prev
    out=[]
    for i,v in enumerate(l):
        if v<0: v=prev[i]
        out.append(v)
    prev=out
    return out

def enc():
    try: return int(lib.read("d7")), int(lib.read("d8"))
    except: return None

x,y = 0.0,0.0
e_last = enc()
COUNTS_PER_UNIT = 700.0

def odom():
    global x,y,e_last
    e = enc()
    if e and e_last:
        d = ((e[0]-e_last[0])+(e[1]-e_last[1]))/2.0/COUNTS_PER_UNIT
        h = math.radians(lib.heading())
        x += d*math.cos(h); y += d*math.sin(h)
        e_last = e
    return x,y

t0=time.time()
state="follow"
found=False
i=0
while time.time()-t0 < 3300:
    i+=1
    l = clean(lib.lidar())
    g, gs = lib.goal()
    radio = lib.read("d5")
    d4 = lib.read("d4"); d9 = lib.read("d9")
    ox,oy = odom()
    h = lib.heading()
    LOG.write(f"{time.time():.1f},{ox:.2f},{oy:.2f},{h:.1f},{','.join(f'{v:.2f}' for v in l)},{gs},{d4},{d9},{radio}\n")
    LOG.flush()
    if g:
        lib.stop()
        print("GOAL!", gs, flush=True)
        found=True
        break
    if radio.strip():
        print("RADIO:", radio, flush=True)
    front = min(l[0], l[1]*1.2, l[15]*1.2)
    right = min(l[12], l[13]*1.2)
    if front < 0.35:
        # turn left in place until front clear
        lib.wheels(-12,12)
        time.sleep(0.3)
        continue
    # right wall follow
    err = right - 0.35
    err = max(-0.3, min(0.3, err))
    turn = err*40   # positive err (too far) -> turn right -> left faster
    base = 30 if front>0.8 else 15
    lft = base+turn; rgt = base-turn
    lib.wheels(round(lft,1), round(rgt,1))
    time.sleep(0.25)
lib.stop()
print("done, found:", found, flush=True)
