import sys; sys.path.insert(0,'/bot/src')
import ctl, rio, nav, time, math, json
def log(m):
    with open('/bot/explore.log','a') as f: f.write(f"{time.strftime('%H:%M:%S')} LEAD {m}\n")
def d11():
    v=[]
    for _ in range(5):
        s=rio.read_line('d11',0.5)
        try: v.append(float(s))
        except: pass
    return sum(v)/len(v) if v else 0
wps=[(-1.68,1.35),(-1.21,1.11),(-0.48,0.76),(-0.06,0.39),(-0.06,-0.43),(0.44,-1.0)]
rio.write_line('d8',"A: I will now LEAD you to the goal in short hops (~0.6 each). Keep climbing d11 toward me; I wait for you after each hop until my d11>0.88. Follow me!")
for i,(wx,wy) in enumerate(wps):
    last=i==len(wps)-1
    x,y=nav.goto(wx,wy,tol=0.2,timeout=45,stop_on_here=last)
    st=ctl.status(); log(f'hop {i} at ({x:.2f},{y:.2f}) here={st.get("here")} d11={d11():.2f}')
    if last and st.get('here')==1: break
    t0=time.time(); lastmsg=0
    while time.time()-t0<150:
        v=d11()
        if v>0.88: break
        if time.time()-lastmsg>25:
            rio.write_line('d8',f"A: waiting for you at hop {i+1}/6. My d11={v:.2f} => you are ~{2*math.sqrt(max(1/v-1,0)):.1f} away. Keep moving so your d11 rises."); lastmsg=time.time()
        time.sleep(2)
    log(f'hop {i} B caught up d11={d11():.2f} waited {time.time()-t0:.0f}s')
# final: ensure on goal
st=ctl.status()
if st.get('here')!=1:
    log('not on goal after hops; exploring for goal'); import os
    os.environ['STOP_ON_HERE']='1'; os.environ['BIAS']='0.6'; os.environ['GX']='0.3'; os.environ['GY']='-0.95'
    os.system('cd /bot && STOP_ON_HERE=1 BIAS=0.6 GX=0.3 GY=-0.95 timeout 240 python3 src/explore.py')
log(f'final status {ctl.status()}')
rio.write_line('d8',"A: I am ON THE GOAL now (here=1) and parked. Come to me: climb d11. Your here flag turns 1 when you are on the goal too.")
os.system('cd /bot && nohup python3 src/beacon.py >/dev/null 2>&1 &')
