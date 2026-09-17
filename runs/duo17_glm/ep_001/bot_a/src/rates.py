import sys,time
sys.path.insert(0,"/bot/src")
from robust import rline
for port in ("d0","d2","d3","d4","d5","d6","d9","d10","d11"):
    t0=time.time(); vals=[]; last=None; changes=0; n=0
    while time.time()-t0<2.0:
        v=rline(port,0.5)
        n+=1
        if v is not None and v!=last:
            changes+=1; last=v
        if len(vals)<3 and v is not None: vals.append(v)
    print(f"{port}: reads={n} changes={changes} e.g. {vals}")
