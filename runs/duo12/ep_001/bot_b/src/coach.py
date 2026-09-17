import time, sys
sys.path.insert(0,'/bot/src')
from lib import r
def med():
    vs=[]
    for _ in range(5):
        try: vs.append(float(r('d11')))
        except: pass
        time.sleep(0.15)
    vs.sort(); return vs[len(vs)//2] if vs else 9
prev=med()
while True:
    cur=med()
    trend="HOTTER (keep direction)" if cur<prev-0.01 else ("COLDER (turn around)" if cur>prev+0.01 else "flat")
    try:
        with open('/dev/robot/d8','w') as f:
            f.write(f"botA COACH d11={cur:.3f} {trend}. Goal=my position. Stop when your here=1.\n")
    except Exception: pass
    prev=cur
    time.sleep(4)
