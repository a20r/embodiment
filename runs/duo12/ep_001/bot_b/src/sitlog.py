import time, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
log=open('map.log','a')
while True:
    st=status(); d=r('d11'); L=lidar()
    log.write(json.dumps({'t':round(time.time(),1),'x':-3.37,'y':4.18,'h':heading(),
        'L':L,'st':st,'d11':d})+'\n'); log.flush()
    if st.get('goal'):
        with open('/bot/src/GOAL_DONE','w') as f: f.write(str(st))
    time.sleep(2)
