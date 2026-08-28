import time, lib
l0 = lib.lidar(); h0 = lib.heading()
lib.wheels(-10, 10)  # expect heading increase
t0=time.time()
while time.time()-t0<15:
    h=lib.heading()
    d=(h-h0)%360
    if 40<d<330: break
    time.sleep(0.2)
lib.stop(); time.sleep(0.5)
l1=lib.lidar(); h1=lib.heading()
print("h0",h0,"h1",h1)
print("l0",l0)
print("l1",l1)
