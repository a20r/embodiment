import rob, walker, time, sys
def ad(x): return (x+180)%360-180
bng=walker.bng
REF=float(sys.argv[1])
DUR=float(sys.argv[2]) if len(sys.argv)>2 else 240
t0=time.time()
walker.turn(ad(0-bng()))
o0=rob.odo(); rob.motors(18,18)
while rob.odo()-o0<450:
    L=rob.lidar()
    if 0<L[0]<0.22: break
    time.sleep(0.05)
n=0
while time.time()-t0<DUR:
    if rob.goal(): print("GOAL!!!", flush=True); break
    b=bng()
    e=ad(b-REF)
    c=max(-14,min(14,0.7*e))
    rob.motors(15+c,15-c)
    n+=1
    if n%80==0: print(f"b={b:.0f}", flush=True)
    time.sleep(0.05)
rob.motors(0,0)
print("end", flush=True)
