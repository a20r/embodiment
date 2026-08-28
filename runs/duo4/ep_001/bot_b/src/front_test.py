import time
def rd(p):
    with open(f"/dev/robot/{p}") as f: return f.readline().strip()
def wr(p,v):
    with open(f"/dev/robot/{p}","w") as f: f.write(str(v)+"\n")

# back off a bit first
wr("d4",-15); wr("d5",-15); time.sleep(1.5)
wr("d4",0); wr("d5",0)
scans=[]
wr("d4",15); wr("d5",15)
t0=time.time()
while time.time()-t0<3:
    r=[float(x) for x in rd("d1").split(",")]
    scans.append(r)
    if rd("d0")=="1": break
    time.sleep(0.1)
wr("d4",0); wr("d5",0)
import statistics
n=len(scans)
print("n=",n)
for i in range(16):
    vals=[s[i] for s in scans if s[i]>0]
    if len(vals)<3: print(i,"na"); continue
    # slope via first/last thirds
    a=statistics.median(vals[:len(vals)//3] or vals[:1])
    b=statistics.median(vals[-(len(vals)//3):] or vals[-1:])
    print(i, round(a,2), round(b,2), round(b-a,2))
