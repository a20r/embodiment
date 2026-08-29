import time
def r(p):
    with open(f'/dev/robot/d{p}') as f: return f.readline().strip()
def w(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
def lidar(): return [float(x) for x in r(3).split(',')]
# find open direction: rotate until beam0 is large
l0=lidar(); print("h",r(1),"l",l0)
t0=time.time()
while time.time()-t0<4:
    w(10,20); w(11,20); time.sleep(0.05)
w(10,0);w(11,0)
time.sleep(0.3)
l1=lidar(); print("h",r(1),"l",l1)
print("diff",[round(b-a,3) for a,b in zip(l0,l1)])
