import sys,time
sys.path.insert(0,"/bot/src")
from robust import rline
f=open("/memory/d11.log","a")
while True:
    v=rline("d11",0.5)
    if v: f.write(f"{time.time():.1f} {v}\n"); f.flush()
    time.sleep(1)
