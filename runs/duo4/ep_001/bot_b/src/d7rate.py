import time,sys
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
def rate(dur):
    n=0;k=0;t0=time.time()
    while time.time()-t0<dur:
        v=rd("d7")
        if v in("0","1"):
            n+=1; k+= (v=="1")
    return k,n
k,n=rate(float(sys.argv[1]) if len(sys.argv)>1 else 8)
print(f"d7 ones {k}/{n}")
