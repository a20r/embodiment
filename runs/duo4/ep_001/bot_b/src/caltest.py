import time
def rd(p):
    with open(f"/dev/robot/{p}") as f: return f.readline().strip()
def wr(p,v):
    with open(f"/dev/robot/{p}","w") as f: f.write(str(v)+"\n")
def scan():
    while True:
        try:
            r=[float(x) for x in rd("d1").split(",")]
            return r
        except: pass
import sys
sp=float(sys.argv[1]); dur=float(sys.argv[2])
r0=scan()[0]; t0=time.time()
wr("d4",sp); wr("d5",sp)
while time.time()-t0<dur:
    if rd("d0")=="1": print("bump"); break
    time.sleep(0.1)
wr("d4",0); wr("d5",0)
t=time.time()-t0
time.sleep(0.2)
r1=scan()[0]
print(f"speed {sp} dur {t:.2f} f0 {r0:.3f}->{r1:.3f} v={(r0-r1)/t:.3f}")
