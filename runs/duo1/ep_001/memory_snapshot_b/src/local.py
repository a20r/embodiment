import lib, time, math
def pingv():
    lib.write("d3","ping"); time.sleep(0.2)
    r=lib.read("d5").strip()
    try: return float(r)
    except: return None
def checkgoal():
    g,gs=lib.goal()
    if g: lib.stop(); print("GOAL!",gs,flush=True); exit()
for it in range(10):
    results={}
    for dd in range(0,360,45):
        h=lib.heading()
        lib.turn_by(((dd-h+180)%360)-180)
        l=lib.lidar()
        if l[0]<0.55:
            print(f"{dd}: blocked",flush=True); continue
        vals=[]
        lib.wheels(25,25)
        for i in range(5):
            v=pingv(); checkgoal()
            if v is not None: vals.append(v)
        lib.stop()
        lib.wheels(-25,-25); time.sleep(5*0.25+0.2); lib.stop()
        checkgoal()
        m=max(vals) if vals else None
        results[dd]=m
        print(f"{dd}: {vals}",flush=True)
    good=[(v,d) for d,v in results.items() if v is not None]
    if not good: print("no signal",flush=True); break
    v,d=max(good)
    print(f"iter {it}: best dir {d} v={v}",flush=True)
    if v>=90: break
    h=lib.heading()
    lib.turn_by(((d-h+180)%360)-180)
    lib.wheels(25,25)
    t0=time.time()
    while time.time()-t0<2.5:
        checkgoal()
        l=lib.lidar()
        if l[0]<0.4: break
        vv=pingv()
        if vv is not None and vv<v-15: break
    lib.stop()
