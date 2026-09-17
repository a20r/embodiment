import os, time, select
for attempt in range(3):
    for n in ['d4','d6','d9','d3']:
        fd=os.open('/dev/robot/'+n, os.O_RDONLY|os.O_NONBLOCK)
        acc=''
        t0=time.time()
        while time.time()-t0<0.35:
            r,_,_=select.select([fd],[],[],0.05)
            if r:
                try:
                    d=os.read(fd,4096).decode()
                    if d: acc+=d
                except BlockingIOError: pass
        os.close(fd)
        print(n, repr(acc[-60:]), flush=True)
    print('---')
