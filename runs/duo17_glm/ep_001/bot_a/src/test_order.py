import sys,time
sys.path.insert(0,"/bot/src")
from nav import *
h0=hd(); print("start h:", h0)
turn(30)  # clockwise: heading decreases
time.sleep(1.5)
turn(0); time.sleep(0.4)
h=hd(); print("h now:", h, "(moved", round(ang_err(h0,h),1),"deg)")
s=sc(); print("scan:", [round(x,2) for x in s])
