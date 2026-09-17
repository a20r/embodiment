import os, select, time, sys
RPORTS=['d0','d2','d3','d4','d5','d6','d9','d11']
dur=float(sys.argv[1]) if len(sys.argv)>1 else 15
t0=time.time()
while time.time()-t0<dur:
    row={}
    for p in RPORTS:
        fd=os.open('/dev/robot/'+p, os.O_RDONLY|os.O_NONBLOCK)
        r,_,_=select.select([fd],[],[],0.05)
        if r:
            try: row[p]=os.read(fd,4096).decode().strip().replace('\n',';')[:60]
            except: row[p]='ERR'
        os.close(fd)
    print(f"{time.time()-t0:6.1f}", row, flush=True)
    time.sleep(0.4)
