import sys,time,json
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop
from nav import hd,sc
from robust import rline
speed(0); turn(0)
out=[]
t0=time.time()
while time.time()-t0<100:
    v=rline("d11",0.2); h=hd(); s=sc()
    try: v=float(v)
    except: v=None
    out.append((round(time.time()-t0,1),h,v,s))
    time.sleep(0.5)
stop()
with open("/memory/watch.out","w") as f:
    for t,h,v,s in out:
        f.write(f"{t} {h} {v} {s}\n")
# summary: rss every 10s, scan movement metric
import statistics
for i in range(0,len(out),20):
    ch=out[i:i+20]
    rss=[c[2] for c in ch if c[2] is not None]
    print(f"t={ch[0][0]:.0f}-{ch[-1][0]:.0f} rss_mean={statistics.mean(rss):.4f}" if rss else "no rss")
# scan variability per beam (stationary robot: walls static, peer moves)
beams=list(zip(*[c[3] for c in out if c[3]]))
print("beam variability (std) while stationary:")
for k,b in enumerate(beams):
    vals=[x for x in b if x>=0]
    if len(vals)>5:
        print(k, round(statistics.mean(vals),2), round(statistics.pstdev(vals),3))
