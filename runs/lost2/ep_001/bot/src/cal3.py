from probe import rd, wr
import time
def scan(): return [float(x) for x in rd(1).split(",")]
# rotate slowly 360, record heading vs front dist
wr(6,"20"); wr(7,"-20")
data=[]
t0=time.time()
while time.time()-t0<25:
    h=float(rd(3)); s=scan()
    data.append((h,s[0]))
wr(6,"0"); wr(7,"0")
data.sort()
# print binned
import math
bins={}
for h,f in data:
    bins.setdefault(int(h//10)*10,[]).append(f)
for k in sorted(bins): print(k, round(max(bins[k]),2))
