#!/usr/bin/env python3
"""Endgame watcher: if other robot beacons goal=1 -> switch to d11 hill-climb homing (cycle bias dirs, keep dir if d11 rising).
If my own goal=1 -> brain already stops; set say.txt accordingly."""
import re, time, json, subprocess
dirs = [0, 90, 180, 270, 45, 135, 225, 315]; di = 0; last = None; homing = False; t_dir = 0
def d11():
    try: return float(json.load(open("/bot/pose.txt"))["d11"])
    except Exception: return None
while True:
    time.sleep(5)
    try: p = json.load(open("/bot/pose.txt"))
    except Exception: continue
    if p.get("goal") == "1":
        open("/bot/say.txt","w").write("GOAL FOUND! goal=1 REACHED THE GOAL. I am PARKED AT GOAL and still. Home in on my signal (d11). When adjacent, come next to me.")
        open("/bot/mode","w").write("stop"); continue
    rx = open("/bot/rx.log").read()
    other_goal = bool(re.search(r"d11=[0-9.]+ goal=1[^0-9]", rx))
    if other_goal and not homing:
        homing = True; subprocess.call("/bot/src/kill_by_name.sh cycle.sh >/dev/null", shell=True)
        open("/bot/say.txt","w").write("Heard your goal=1. HOMING toward you via d11 now. Stay parked at goal.")
        last = d11(); t_dir = time.time(); open("/bot/mode","w").write(f"bias {dirs[di]}")
    if homing:
        v = d11()
        if v is not None and v > 0.93:
            open("/bot/mode","w").write("stop"); open("/bot/say.txt","w").write("I am adjacent to you (d11>0.93) and parked. Are we both on the goal? Tell me which direction to shift if needed.")
            continue
        if time.time() - t_dir > 40:
            if v is not None and last is not None and v < last - 0.02:
                di = (di + 1) % len(dirs)
            last = v; t_dir = time.time(); open("/bot/mode","w").write(f"bias {dirs[di]}")
