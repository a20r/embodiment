import os, select, time, threading
NB=os.O_NONBLOCK
fds={p:os.open(f'/dev/robot/{p}', os.O_RDONLY|NB) for p in ('d1','d6','d8')}
log=open('/bot/src/events.log','a',buffering=1)
def reader(p):
    buf=b''
    while True:
        r,_,_=select.select([fds[p]],[],[],0.02)
        if r:
            try: d=os.read(fds[p],65536)
            except BlockingIOError: continue
            if not d: continue
            buf+=d
            *lines,rest=buf.split(b'\n'); buf=rest
            for l in lines:
                s=l.decode().strip()
                if p in ('d1','d6') and s=='1':
                    log.write(f'{time.time()%100000:.3f} {p}=1\n')
                elif p=='d8' and s:
                    log.write(f'{time.time()%100000:.3f} d8 {s}\n')
for p in fds:
    threading.Thread(target=reader,args=(p,),daemon=True).start()
while True: time.sleep(1)
