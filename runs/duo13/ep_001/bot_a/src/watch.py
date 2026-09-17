import sys, time, json
sys.path.insert(0,'/bot/src')
from robot import R
r=R()
f=open('/memory/watch.log','a',buffering=1)
while True:
    try:
        st=r.status() or (0,0,0)
        d0=r.read(0,0.03); d5=r.read(5,0.03); d11=r.read(11,0.03)
        h=r.heading()
        try: pose=json.load(open('/memory/pose.json'))
        except: pose={}
        f.write(f"[{time.time():.0f}] tick={st[0]} goal={st[1]} here={st[2]} d0={d0} d5={d5} d11={d11} h={h} pose=({pose.get('x',0):.2f},{pose.get('y',0):.2f})\n")
    except Exception as e:
        f.write(f"err {e!r}\n")
    time.sleep(1.0)
