import lib, time
def probe(hdgdeg):
    h=lib.heading()
    lib.turn_by(((hdgdeg-h+180)%360)-180)
    l=lib.lidar()
    if l[0]<0.7:
        print(f"dir {hdgdeg}: blocked f={l[0]}", flush=True); return
    lib.wheels(30,30)
    vals=[]
    for i in range(6):
        lib.write("d3","ping"); time.sleep(0.25)
        r=lib.read("d5").strip()
        if r:
            try: vals.append(float(r))
            except: vals.append(r)
    lib.stop()
    lib.wheels(-30,-30); time.sleep(6*0.30); lib.stop()
    print(f"dir {hdgdeg}: {vals}", flush=True)
for d in (0,90,180,270):
    probe(d)
