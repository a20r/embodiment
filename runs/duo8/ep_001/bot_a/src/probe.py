import time
def r(p):
    with open(f'/dev/robot/d{p}') as f: return f.readline().strip()
def w(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
print("start h=",r(1),"lidar=",r(3))
t0=time.time()
while time.time()-t0<4:
    w(10,1.0); w(11,1.0); time.sleep(0.1)
print("after fwd h=",r(1),"lidar=",r(3))
t0=time.time()
while time.time()-t0<3:
    w(10,1.0); w(11,-1.0); time.sleep(0.1)
print("after turn h=",r(1),"lidar=",r(3))
w(10,0);w(11,0)
