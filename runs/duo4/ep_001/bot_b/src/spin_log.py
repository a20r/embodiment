import time, json
def rd(p):
    with open(f"/dev/robot/{p}") as f: return f.readline().strip()
def wr(p,v):
    with open(f"/dev/robot/{p}","w") as f: f.write(str(v)+"\n")
log=[]
wr("d4",-20); wr("d5",20)   # rotate CCW ~ +37deg/s
t0=time.time()
while time.time()-t0<11:
    try:
        h=float(rd("d2")); r=[float(x) for x in rd("d1").split(",")]
        log.append((time.time()-t0,h,r))
    except: pass
    time.sleep(0.08)
wr("d4",0); wr("d5",0)
json.dump(log, open("/tmp/spin.json","w"))
print(len(log), log[0][1], log[-1][1])
