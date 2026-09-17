import os, select, time, sys
dur=float(sys.argv[1]) if len(sys.argv)>1 else 10
t0=time.time(); d6s=[]; d9s=[]
while time.time()-t0<dur:
    fd=os.open('/dev/robot/d6', os.O_RDONLY|os.O_NONBLOCK)
    r,_,_=select.select([fd],[],[],0.05)
    if r:
        try: d6s.append((time.time()-t0,float(os.read(fd,4096))))
        except: pass
    os.close(fd)
    fd=os.open('/dev/robot/d9', os.O_RDONLY|os.O_NONBLOCK)
    r,_,_=select.select([fd],[],[],0.05)
    if r:
        try: d9s.append((time.time()-t0,float(os.read(fd,4096))))
        except: pass
    os.close(fd)
    time.sleep(0.2)
def slope(a):
    if len(a)<2: return None
    (t1,v1),(t2,v2)=a[0],a[-1]
    return (v2-v1)/(t2-t1), v1, v2, len(a)
print("d6 slope/vals:", slope(d6s))
print("d9 slope/vals:", slope(d9s))
