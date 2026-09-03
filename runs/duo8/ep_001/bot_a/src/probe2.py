import time
def r(p):
    with open(f'/dev/robot/d{p}') as f: return f.readline().strip()
def w(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
for val in [5, 20, 100]:
    h0=float(r(1)); t0=time.time()
    while time.time()-t0<2:
        w(10,val); w(11,-val); time.sleep(0.05)
    h1=float(r(1))
    print(f"val={val} dh={(h1-h0)%360}")
w(10,0);w(11,0)
print("lidar",r(3))
