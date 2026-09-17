import sys,time; sys.path.insert(0,'/bot/src')
from explore import *
L=open('/memory/autopilot_log.txt','a')
def lg(*a):
    s=time.strftime('%H:%M:%S ')+' '.join(map(str,a)); L.write(s+'\n'); L.flush()
lastmsg=0; close_since=None; nrx=sum(1 for _ in open('/tmp/rx.txt'))
while True:
    st=status(); sg=sig() or 0
    n=sum(1 for _ in open('/tmp/rx.txt'))
    if n>nrx:
        for line in list(open('/tmp/rx.txt'))[nrx:]: lg("RX:",line.strip()[:250])
        nrx=n
    if st.get('goal')=='1': lg("GOAL FLAG =1 !!! DONE", st); writeline(8,"R2: goal flag=1, we did it!"); time.sleep(30); continue
    if time.time()-lastmsg>45:
        writeline(8,f"R2 {time.strftime('%H:%M')}: I am INSIDE the GOAL (here=1), parked NORTH side. Goal = origin +E8 N0 (+-1N), dead-end cell entered from its west. You know it. Come in SOUTH side. sig={sg:.2f}"); lastmsg=time.time()
    if sg>0.95:
        if close_since is None: close_since=time.time(); lg("robot1 very close, sig",sg,st)
        elif time.time()-close_since>60 and st.get('here')=='1':
            lg("re-entering goal zone to refresh arrival"); turn_to(270,tol=4)
            v,h,sc=cardinal_view(); f=v[270]
            if f>0.5: drive_front(max(0.2,f-0.35),270,maxt=15); time.sleep(2); turn_to(90,tol=4); v,h,sc=cardinal_view(); drive_front(0.25,90,maxt=15)
            lg("after re-entry", status()); close_since=time.time()
    else: close_since=None
    if int(time.time())%300<3: lg("status",st,"sig",sg)
    time.sleep(3)
