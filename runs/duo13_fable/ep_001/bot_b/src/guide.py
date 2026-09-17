from rob import *
import time
L=open('/bot/src/guide.txt','a')
last=0; hist=[]
while True:
    s=rd('d11',0.5); st=rd('d3',0.5)
    try: s=float(s)
    except: s=None
    if s is not None: hist.append(s)
    L.write(f"{time.strftime('%H:%M:%S')} sig={s} {st}\n"); L.flush()
    if st and 'goal=1' in st: L.write("!!! SUCCESS goal=1\n"); L.flush()
    if time.time()-last>40 and s is not None:
        trend='RISING' if len(hist)>8 and hist[-1]>hist[-8]+0.03 else ('FALLING' if len(hist)>8 and hist[-1]<hist[-8]-0.03 else 'flat')
        wr('d8',f"R1: I am ON the goal (here=1), waiting. Our signal now {s:.2f} ({trend}). Move so it rises toward 1.0. Reply with your sig + open dirs (N/E/S/W) and I will route you.")
        last=time.time()
    time.sleep(5)
