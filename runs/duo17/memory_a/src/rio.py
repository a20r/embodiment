import os, time, select, sys
DEV='/dev/robot/'
def rd(p, to=1.0):
    """read one line from port p with timeout"""
    try:
        fd = os.open(DEV+p, os.O_RDONLY|os.O_NONBLOCK)
    except OSError as e:
        return None
    buf=b''; t0=time.time()
    try:
        while time.time()-t0<to:
            r,_,_=select.select([fd],[],[],0.05)
            if r:
                c=os.read(fd,4096)
                if not c:
                    if buf: break
                    time.sleep(0.01); continue
                buf+=c
                if b'\n' in buf: break
    finally:
        os.close(fd)
    return buf.decode(errors='replace').strip()
def wr(p, s, to=1.0):
    t0=time.time()
    while time.time()-t0<to:
        try:
            fd=os.open(DEV+p, os.O_WRONLY|os.O_NONBLOCK)
        except OSError:
            time.sleep(0.02); continue
        try:
            os.write(fd,(s+'\n').encode())
        finally:
            os.close(fd)
        return True
    return False
def snap():
    d={p:rd(p) for p in ['d0','d3','d4','d5','d6','d9','d11']}
    d['d2']=rd('d2')
    return d
if __name__=='__main__':
    import json
    if sys.argv[1]=='r': print(rd(sys.argv[2]))
    elif sys.argv[1]=='w': print(wr(sys.argv[2], sys.argv[3]))
    elif sys.argv[1]=='snap': print(json.dumps(snap()))
