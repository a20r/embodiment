import sys, time
sys.path.insert(0,'/bot/src')
from rio import *
def snap():
    r=ranges(); h=heading(); return r,h
for port in (1,7):
    for cmd in ["1","1 1","1,1","forward","F","10","0.5 0.5","-1"]:
        r0,h0=snap()
        writeline(port,cmd)
        time.sleep(1.5)
        r1,h1=snap()
        d=max(abs(a-b) for a,b in zip(r0,r1)) if r0 and r1 else -1
        print(f"d{port} <- {cmd!r}: maxdelta_range={d:.3f} heading {h0}->{h1} beam4 {r0[4]:.2f}->{r1[4]:.2f}", flush=True)
        writeline(port,"0")
        time.sleep(0.5)
