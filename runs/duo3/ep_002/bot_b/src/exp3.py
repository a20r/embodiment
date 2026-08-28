from drv import Bot
import time
b=Bot()
time.sleep(0.5)
for cmd in [45,45,90,22.5,-22.5,-90]:
    h0=b.heading(); b.turn(cmd); time.sleep(0.5); h1=b.heading()
    d=(h1-h0)%360
    if d>180: d-=360
    print(cmd,h0,h1,round(d,1))
