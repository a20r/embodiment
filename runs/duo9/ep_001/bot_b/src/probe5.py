import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
def snap(label):
    print(label, "hdg",read_port("d1"), "d7",read_port("d7"), "d8",read_port("d8"), "d2",read_port("d2"), "d5",read_port("d5"), flush=True)
for i in range(3): write_port("d10","0"); write_port("d11","0")
for i in range(6):
    snap(f"rest {i}"); time.sleep(1)
