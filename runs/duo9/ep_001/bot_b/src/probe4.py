import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
def hdg():
    try: return float(read_port("d1"))
    except: return None
# ensure stopped: write zeros
for i in range(3):
    write_port("d10","0"); write_port("d11","0")
time.sleep(1)
print("start hdg", hdg(), "enc", read_port("d7"), read_port("d8"), flush=True)
# single impulse to d10
write_port("d10","1.0")
for i in range(8):
    time.sleep(0.5)
    print(f"t={i*0.5+0.5}s hdg={hdg()} enc7={read_port('d7')} enc8={read_port('d8')}", flush=True)
