import rob, walker, time, sys
bng=walker.bng
def ad(x): return (x+180)%360-180
REF=float(sys.argv[1]) if len(sys.argv)>1 else 65.0
t0=time.time()
# first: face goal and attach to block wall
walker.turn(ad(0-bng()))
o0=rob.odo(); rob.motors(18,18)
while rob.odo()-o0<400:
    L=rob.lidar()
    if 0<L[0]<0.22: break
    time.sleep(0.05)
rob.motors(0,0)
print("attached F=%.2f b=%.0f"%(rob.lidar()[0], bng()), flush=True)
n=0
while time.time()-t0<280:
    if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); break
    b=bng()
    e=ad(b-REF)
    c=max(-12,min(12,0.6*e))
    rob.motors(16+c,16-c)
    n+=1
    if n%40==0: print(f"b={b:.0f}", flush=True)
    time.sleep(0.05)
rob.motors(0,0)
print("crawl end", flush=True)
