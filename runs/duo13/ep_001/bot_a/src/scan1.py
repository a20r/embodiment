import sys, time; sys.path.insert(0,'/bot/src')
from robot import R
r=R(); import json
log=[]
r.motors(0,0); time.sleep(0.5)
prev=r.enc()
for step in range(14):
    r.motors(80,-80)      # spin
    time.sleep(0.30)
    r.motors(0,0); time.sleep(0.25)
    h=r.heading(); rg=r.ranges(); enc=r.enc(); st=r.status()
    log.append({"step":step,"h":h,"rg":rg,"enc":enc,"st":st})
    print(step, h, [None if x is None else round(x,2) for x in rg] if rg else None, flush=True)
r.motors(0,0)
json.dump(log, open('/memory/scan1.json','w'))
print("saved")
