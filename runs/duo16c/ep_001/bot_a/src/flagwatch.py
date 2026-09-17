import os, select, time
LOG=open('/memory/flags.txt','a',buffering=1)
last=(0,0)
n=0
while True:
    try:
        fd=os.open('/dev/robot/d3',os.O_RDONLY)
        r,_,_=select.select([fd],[],[],0.1)
        if r:
            d=os.read(fd,4096).decode().strip()
            try:
                parts=dict(kv.split('=') for kv in d.split() if '=' in kv)
                cur=(int(parts.get('goal','0')), int(parts.get('here','0')))
                if cur!=last:
                    LOG.write(f"[{time.time():.0f}] FLAGS {last}->{cur} tick={parts.get('tick')}\n")
                    last=cur
            except Exception: pass
        os.close(fd)
    except Exception:
        time.sleep(0.2)
    n+=1
    if n%2000==0:
        LOG.write(f"[{time.time():.0f}] alive, flags={last}\n")
