import lib, time
def enc(): return int(lib.read("d7")), int(lib.read("d8"))
e0=enc(); f0=lib.lidar()[0]; h0=lib.heading()
lib.wheels(25,25)
t0=time.time()
while time.time()-t0<20:
    l=lib.lidar()
    if l[0]<0.5 and l[0]>0: break
    time.sleep(0.3)
lib.stop(); time.sleep(0.3)
e1=enc(); f1=lib.lidar()[0]; h1=lib.heading()
print("f",f0,"->",f1,"moved",f0-f1)
print("enc",e1[0]-e0[0],e1[1]-e0[1])
print("hdg",h0,"->",h1)
print("counts per dist:", (e1[0]-e0[0])/(f0-f1))
