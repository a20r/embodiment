import sys, os, select, time
sys.path.insert(0,'/bot/src')
fd_tx = os.open('/dev/robot/d8', os.O_WRONLY)   # hold open? try per-write
fd_rx = os.open('/dev/robot/d10', os.O_RDONLY)
i=0
log=open('/memory/rx.log','a')
while True:
    r,_,_ = select.select([fd_rx],[],[],1.0)
    if r:
        d = os.read(fd_rx, 4096)
        if d.strip():
            log.write('%.1f RX %r\n'%(time.time()%10000, d)); log.flush()
    i+=1
    if i%2==0:
        try:
            w = os.open('/dev/robot/d8', os.O_WRONLY)
            os.write(w, b'PING A tick\n')
            os.close(w)
        except Exception as e:
            pass
