import os,time,sys
sys.path.insert(0,'/bot/src')
# lightweight RX+d6 logger + beacon (no wheel use)
DEV='/dev/robot/'
def op(n): return os.open(DEV+n, os.O_RDONLY|os.O_NONBLOCK)
rx=op('d4'); d6=op('d6'); d5=op('d5')
tx=os.open(DEV+'d0', os.O_WRONLY)
bufs={rx:b'',d6:b'',d5:b''}
log=open('/memory/ep3_bg.log','a')
def L(s):
    log.write('%.1f %s\n'%(time.time(),s)); log.flush()
last={rx:None,d6:None,d5:None}
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
L('=== bg start ===')
lastb=0; lastd5log=0
while True:
    for m in poll(rx): L('RX: '+m)
    for m in poll(d6):
        if 'goal=1' in m: L('GOAL1: '+m)
    poll(d5)
    now=time.time()
    if now-lastd5log>5:
        L('d5=%s'%last[d5]); lastd5log=now
    if now-lastb>3:
        os.write(tx,b'A here. B: PARK AND SPIN NOW at your position. I will home on d5 sound to you. Co-location test is priority.\n')
        lastb=now
    time.sleep(0.05)
