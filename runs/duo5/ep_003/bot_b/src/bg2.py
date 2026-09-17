import os,time
DEV='/dev/robot/'
def op(n): return os.open(DEV+n, os.O_RDONLY|os.O_NONBLOCK)
d6=op('d6'); d5=op('d5')
bufs={d6:b'',d5:b''}; last={d6:None,d5:None}
log=open('/memory/ep3_bg.log','a')
def L(s): log.write('%.1f %s\n'%(time.time(),s)); log.flush()
def poll(fd):
    lines=[]
    try:
        while True:
            d=os.read(fd,65536)
            if not d: break
            bufs[fd]+=d
    except BlockingIOError: pass
    while b'\n' in bufs[fd]:
        l,bufs[fd]=bufs[fd].split(b'\n',1)
        s=l.decode(errors='replace').strip()
        if s: lines.append(s); last[fd]=s
    return lines
L('=== bg2 start (no TX, no RX-consume... wait RX consumed by dc) ===')
t=0
while True:
    for m in poll(d6):
        if 'goal=1' in m: L('GOAL1: '+m)
    poll(d5)
    if time.time()-t>5: L('d5=%s'%last[d5]); t=time.time()
    time.sleep(0.05)
