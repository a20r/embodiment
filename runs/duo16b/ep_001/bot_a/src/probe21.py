import os, select, time
D='/dev/robot/'
def read(p, timeout=0.2):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,200000).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,msg):
    if isinstance(msg,(int,float)): msg=f"{msg}\n"
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,msg.encode()); os.close(fd)
    except Exception: pass

def sample(label, n=4):
    out=[]
    for i in range(n):
        out.append((read('d5'), read('d0'), read('d11')))
        time.sleep(0.3)
    print(label, out)

sample("stationary:", 4)
w('d1',80); w('d7',80); sample("fwd80:", 4)
w('d1',-80); w('d7',-80); sample("rev80:", 4)
w('d1',-80); w('d7',80); sample("spinL:", 3)
w('d1',0); w('d7',0); sample("stop:", 3)
