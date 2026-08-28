import lib, time
def enc():
    return int(lib.read("d7")), int(lib.read("d8"))
l=lib.lidar(); e0=enc(); print("f0",l[0],"b0",l[8],"enc",e0)
lib.wheels(20,20); time.sleep(3); lib.stop(); time.sleep(0.3)
l=lib.lidar(); e1=enc(); print("f1",l[0],"b1",l[8],"enc",e1)
print("dcounts", e1[0]-e0[0], e1[1]-e0[1])
