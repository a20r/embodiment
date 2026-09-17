import time, math
D='/dev/robot/'
def readl(p,tries=2):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read()
            lines=[x.strip() for x in s.split('\n') if x.strip()]
            if lines: return lines[-1]
        except Exception: pass
        time.sleep(0.01)
    return None
def fnum(p):
    s=readl(p)
    try: return float(s)
    except: return None
KM=0.006; TPS=5.3
pose={'x':0.0,'y':0.0,'t':time.time(),'r':None,'l':None,'h':None}
F=open('/memory/tri3.csv','a')
while True:
    r=fnum('d6'); l=fnum('d9'); h=fnum('d4'); v=fnum('d11')
    now=time.time()
    if pose['r'] is not None and None not in (r,l,h,v) and pose['h'] is not None:
        dt=now-pose['t']
        if dt>0:
            ds=((r-pose['r'])+(l-pose['l']))/2.0/TPS*KM
            dh=(h-pose['h']+180)%360-180
            hm=math.radians(pose['h']+dh/2.0)
            pose['x']+=ds*math.sin(hm); pose['y']+=ds*math.cos(hm)
    pose['r']=r; pose['l']=l; pose['h']=h; pose['t']=now
    F.write(f"{now:.2f},{pose['x']:.3f},{pose['y']:.3f},{h:.1f},{v:.3f}\n")
    F.flush()
    time.sleep(0.4)
