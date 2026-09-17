import sys; sys.path.insert(0,'/bot/src')
import rio, time, math
while True:
    d11 = rio.read_line('d11', 1.0)
    try: dist = 2*math.sqrt(1/float(d11)-1)
    except: dist = -1
    rio.write_line('d8', f"A: parked on goal, not moving. My d11={d11} => you are ~{dist:.1f} units from me. Come closer: move so your d11 rises. Report your d11 + compass heading.")
    time.sleep(25)
