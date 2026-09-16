import os, select, time
LOG = open('/memory/rxlog.txt','a', buffering=1)
n=0
while True:
    try:
        fd = os.open('/dev/robot/d10', os.O_RDONLY)
        r,_,_ = select.select([fd],[],[],0.5)
        if r:
            d = os.read(fd, 8192).decode(errors='replace').strip()
            if d:
                LOG.write(f"[{time.time():.0f}] RX: {d}\n")
                n=0
            else:
                n+=1
                if n==2000:
                    LOG.write(f"[{time.time():.0f}] (2000 empty reads)\n"); n=0
        os.close(fd)
    except Exception as e:
        LOG.write(f"[{time.time():.0f}] ERR {e}\n"); time.sleep(1)
