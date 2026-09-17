import sys,time
sys.path.insert(0,"/bot/src")
from nav import *
print("h before:", hd())
turn_to(82)
time.sleep(0.3)
print("h after turn:", hd())
s=sc(); print("scan:", [round(x,2) for x in s])
speed(2)
time.sleep(2.5)
speed(0); time.sleep(0.4)
print("h after fwd:", hd())
s=sc(); print("scan:", [round(x,2) for x in s])
