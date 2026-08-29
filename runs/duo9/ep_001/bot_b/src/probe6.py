import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
def snap(label):
    print(label, "hdg",read_port("d1"), "d7",read_port("d7"), "d8",read_port("d8"), flush=True)
snap("start")
write_port("d10","10")
for i in range(5): time.sleep(1); snap(f"d10=10 t={i+1}")
write_port("d10","0")
snap("stopped")
write_port("d11","10")
for i in range(5): time.sleep(1); snap(f"d11=10 t={i+1}")
write_port("d11","0")
snap("stopped")
