import time
def rd(p):
    with open(f"/dev/robot/{p}") as f: return f.readline().strip()
def wr(p,v):
    with open(f"/dev/robot/{p}","w") as f: f.write(str(v)+"\n")
def scan():
    while True:
        try: return [float(x) for x in rd("d1").split(",")]
        except: pass
# rotate until idx4 is the max beam (face opening), then drive
best=None
wr("d4",-15); wr("d5",15)
t0=time.time()
while time.time()-t0<12:
    r=scan()
    if r[0]>=2.0: break
    time.sleep(0.05)
wr("d4",0); wr("d5",0)
time.sleep(0.3)
r=scan(); print("faced: idx4=",r[4], r)
h0=float(rd("d2"))
wr("d4",15); wr("d5",15)
t0=time.time()
while time.time()-t0<4:
    r=scan()
    print(round(time.time()-t0,1), "f0=",r[0], "b8=",r[8], "bump=",rd("d0"))
    if rd("d0")=="1": break
    time.sleep(0.3)
wr("d4",0); wr("d5",0)
print("dh=", float(rd("d2"))-h0)
