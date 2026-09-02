import sys, time, json; sys.path.insert(0,'/bot/src')
from robot import R
r=R(); r.motors(0,0); time.sleep(0.4)
log=[]
for step in range(14):
    r.motors(80,-80); time.sleep(0.30); r.motors(0,0); time.sleep(0.25)
    log.append({"h":r.heading(),"rg":r.ranges(),"st":r.status()})
    print(step, r.heading(), [round(x,2) if x else x for x in (r.ranges() or [])], flush=True)
r.motors(0,0)
json.dump(log, open('/memory/scan2.json','w'))
