import time, math, json, os, threading, sys
sys.path.insert(0,"/bot/src")
exec(open("home.py").read().split('if __name__')[0])
# provides: r, rssi(), drive(), turn_to(), motors, upd, x,y, log
def srssi():
    time.sleep(1.6); return rssi() or 0
best_h=None
for it in range(40):
    base=srssi()
    for m in r.get_rx(): log("RX:",m)
    log(f"iter {it} pose=({x:.2f},{y:.2f}) rssi={base:.3f}")
    json.dump({"x":x,"y":y},open("/memory/pose.json","w"))
    if base>2.5: log("VERY CLOSE"); break
    moved=False
    hs=[best_h] if best_h is not None else []
    hs+= [0,90,180,270]
    tried=set()
    for h in hs:
        if h in tried: continue
        tried.add(h)
        res,dn=drive(h,1.6,fast=150)
        v=srssi()
        if v>base*1.10:
            log(f"  h={h} v={v:.3f} GOOD"); best_h=h; moved=True; break
        elif res!="done" or v<base*0.9:
            # go back
            drive((h+180)%360,dn,fast=150)
            log(f"  h={h} v={v:.3f} back (res={res})")
        else:
            log(f"  h={h} v={v:.3f} keep"); moved=True; best_h=h; break
    if not moved:
        log("no improvement any direction; stopping"); break
