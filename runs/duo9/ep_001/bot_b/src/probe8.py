import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
motors(0,0)
h0=read_float("d1"); l0=lidar(); e70=read_float("d7"); e80=read_float("d8")
print("h0",h0); print("l0",l0, flush=True)
motors(10,-10)
t0=time.time()
while True:
    h=read_float("d1")
    d=(h-h0)%360
    if 80<d<100 or time.time()-t0>20: break
    time.sleep(0.1)
motors(0,0)
time.sleep(0.5)
h1=read_float("d1"); l1=lidar(); e71=read_float("d7"); e81=read_float("d8")
print("h1",h1,"turn took",round(time.time()-t0,1),"s")
print("l1",l1, flush=True)
print("denc", e71-e70, e81-e80)
