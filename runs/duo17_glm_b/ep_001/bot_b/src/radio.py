import sys, time, json
sys.path.insert(0,'/bot/src')
from robot import rd, wr, status

LOG='/memory/radio.log'
def log(s):
    with open(LOG,'a') as f:
        f.write(s+'\n')

# non-blocking-ish listen thread via select on fifo? simpler: open d10 with O_NONBLOCK
import os, select
flags = os.O_RDONLY | os.O_NONBLOCK
fd10 = os.open('/dev/robot/d10', flags)
t0=time.time()
seq=0
while True:
    seq+=1
    try:
        os.write(fd_ok:=os.open('/dev/robot/d8', os.O_WRONLY|os.O_NONBLOCK), f'PING A {seq}\n'.encode())
        os.close(fd_ok)
    except Exception as e:
        pass
    st = status()
    r,w,_ = select.select([fd10],[],[],0.5)
    if r:
        try:
            data = os.read(fd10, 4096).decode(errors='replace').strip()
            if data: log(f'{time.time():.1f} RX: {data}')
        except Exception as e:
            log(f'{time.time():.1f} RXERR {e}')
    if seq % 5 == 0:
        log(f'{time.time():.1f} ST {st} batt={rd("d11")} d0={rd("d0")} d5={rd("d5")}')
    time.sleep(1.0)
