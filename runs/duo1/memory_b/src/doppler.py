import lib, time
def drain():
    import select
    while True:
        r=lib.read("d5")
        if not r.strip(): break
def probe(hdgdeg):
    h=lib.heading()
    delta=((hdgdeg-h+180)%360)-180
    lib.turn_by(delta)
    l=lib.lidar()
    if min(l[0],l[1]*1.3,l[15]*1.3)<0.5:
        print(f"dir {hdgdeg}: blocked", flush=True); return
    lib.wheels(30,30)
    vals=[]
    for i in range(6):
        lib.write("d3","ping"); time.sleep(0.25)
        r=lib.read("d5").strip()
        if r: vals.append(float(r))
    lib.stop()
    # back up same amount
    lib.wheels(-30,-30); time.sleep(6*0.28); lib.stop()
    print(f"dir {hdgdeg}: {vals}", flush=True)
for d in (0,90,180,270):
    probe(d)
