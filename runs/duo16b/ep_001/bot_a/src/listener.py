import os, select, time, sys
D='/dev/robot/'
logf = open('/memory/radio_log.txt','a', buffering=1)
fd = os.open(D+'d10', os.O_RDONLY | os.O_NONBLOCK)
buf = b''
while True:
    r,_,_ = select.select([fd],[],[],1.0)
    if r:
        try:
            data = os.read(fd, 4096)
            if data:
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n',1)
                    if line.strip():
                        logf.write(f"{time.time():.1f} RX: {line.decode(errors='replace')}\n")
        except Exception as e:
            pass
