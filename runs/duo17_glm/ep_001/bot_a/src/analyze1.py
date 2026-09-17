import math
samples=[]
for line in open("/memory/scan1.txt"):
    p=line.split()
    t=float(p[0]); h=float(p[1]); s=[float(x) for x in p[2].split(",")]
    samples.append((t,h,s))
# assume beams at h + k*22.5 (offset 0 for now); build polar profile of nearest returns
# For each beam index, collect (world_angle mod 360, range)
print("beam-index range summary across spin (min/median):")
import statistics
for k in range(16):
    vals=[s[k] for _,_,s in samples if s[k]>0]
    print(k, round(min(vals),2), round(statistics.median(vals),2), round(max(vals),2))
print()
# histogram: world angle (offset 0) -> min range
def profile(offset):
    bins={}
    for t,h,s in samples:
        for k in range(16):
            v=s[k]
            if v<0: continue
            a=(h+offset+k*22.5)%360
            b=int(a//10)*10
            bins[b]=min(bins.get(b,99),v)
    return bins
for off in (0,):
    b=profile(off)
    print("world-angle (deg, offset",off,") -> min range")
    for a in sorted(b): print(f"  {a:3d}-{a+10:3d}: {b[a]:.2f}")
