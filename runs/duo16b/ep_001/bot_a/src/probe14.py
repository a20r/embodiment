import os, select, time

D = '/dev/robot/'
def read(p, timeout=0.2):
    fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd], [], [], timeout)
    out = ''
    if r:
        try: out = os.read(fd, 200000).decode().strip()
        except Exception: out = ''
    os.close(fd)
    return out

def w(p, msg):
    if isinstance(msg,(int,float)): msg = f"{msg}\n"
    try:
        fd = os.open(D+p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, msg.encode()); os.close(fd)
    except Exception as e: print("werr", p, e)

# back up a bit first (we're close to something)
t0=time.time()
while time.time()-t0<1.0:
    w('d1',-50); w('d7',-50); time.sleep(0.1)
w('d1',0); w('d7',0); time.sleep(0.3)
print("backed up: lidar", end=' ')
s = read('d2'); pts=[p for p in s.split(';') if p.strip()]
rs=[float(p.split(',')[0]) for p in pts if p.split(',')[0]!='' and float(p.split(',')[0])>0]
print("%.3f" % (sum(rs)/len(rs)))

# rotate 360 slowly, log heading & d5 & d0
t0=time.time(); log=[]
while time.time()-t0 < 12.0:
    w('d1',25); w('d7',-25)
    time.sleep(0.35)
    log.append((read('d4'), read('d5'), read('d0'), read('d11')))
w('d1',0); w('d7',0)
ones = [(h, d11) for h,d5,d0,d11 in log if d5=='1']
zeros = [h for h,d5,d0,d11 in log if d5=='0']
print("d5=1 headings:", [h for h,_ in ones])
print("d5=0 sample:", zeros[:10], "...", zeros[-5:])
print("d11 when d5=1:", [d for _,d in ones])
print("d0 any 1?:", any(d0=='1' for _,_,d0,_ in log))
