import lib, time, math
# odometry resume
last=None
for line in open("/memory/trail2.csv"): last=line
p=last.split(","); x,y=float(p[1]),float(p[2])
def enc():
    try: return int(lib.read("d7")), int(lib.read("d8"))
    except: return (0,0)
el=enc()
LOG=open("/memory/trail2.csv","a")
def odom():
    global x,y,el
    e=enc()
    d=((e[0]-el[0])+(e[1]-el[1]))/2.0/700.0
    h=math.radians(lib.heading())
    x+=d*math.cos(h); y+=d*math.sin(h); el=e
best=(-999,None,None)
def step():
    global best
    lib.write("d3","ping"); time.sleep(0.18)
    r=lib.read("d5").strip()
    odom()
    g,gs=lib.goal()
    d4=lib.read("d4"); d9=lib.read("d9")
    v=None
    try: v=float(r)
    except: pass
    LOG.write(f"{time.time():.1f},{x:.2f},{y:.2f},{lib.heading():.1f},{','.join(['0']*16)},{gs},{d4},{d9},{r}\n"); LOG.flush()
    if v is not None and v>best[0]:
        best=(v,x,y)
        print(f"new best v={v} at {x:.2f},{y:.2f} d4={d4} d9={d9} {gs}",flush=True)
    if g:
        lib.stop(); print("GOAL!",gs,flush=True); exit()
    return v
def drivedir(hdg, dur):
    lib.turn_by(((hdg-lib.heading()+180)%360)-180)
    lib.wheels(18,18)
    t0=time.time(); hold=hdg
    while time.time()-t0<dur:
        l=lib.lidar()
        if l[0]<0.35: break
        e=((hold-lib.heading()+180)%360)-180
        c=max(-6,min(6,e*0.5))
        lib.wheels(round(18-c,1),round(18+c,1))
        step()
    lib.stop()
for cycle in range(4):
    drivedir(90, 12)   # north slow
    drivedir(270, 12)  # south slow
print("BEST:",best,flush=True)
