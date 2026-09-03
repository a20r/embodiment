import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
for sp in (50, 200):
    e0=read_float("d7")
    motors(sp,-sp)
    time.sleep(2)
    motors(0,0)
    e1=read_float("d7")
    print(f"cmd {sp}: enc rate {(e1-e0)/2}/s", flush=True)
