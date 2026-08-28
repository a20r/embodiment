import sys, time
sys.path.insert(0,'/memory/code')
from robot import Robot
r = Robot('/bot/src/sensors.log')
time.sleep(1)
def avg_scan(n=6):
    acc=[[] for _ in range(16)]
    for _ in range(n):
        s=r.scan()
        if s:
            for i,x in enumerate(s):
                if x>0: acc[i].append(x)
        time.sleep(0.3)
    return [round(sum(a)/len(a),3) if a else -1 for a in acc]
print("heading", r.heading(), flush=True)
print("scan", avg_scan(), flush=True)
for p in [2,5,10]:
    s0=avg_scan(4)
    r.cmd(0,p); time.sleep(3); r.stop(); time.sleep(0.7)
    s1=avg_scan(4)
    d=[round(b-a,3) for a,b in zip(s0,s1)]
    print("throttle",p,"delta",d, flush=True)
    r.cmd(0,-p); time.sleep(3); r.stop(); time.sleep(0.7)
print("final scan", avg_scan(4), "heading", r.heading(), flush=True)
