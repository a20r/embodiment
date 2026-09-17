import os, time, select

def readp(p, tmo=0.2):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_RDONLY | os.O_NONBLOCK)
        r,_,_ = select.select([fd],[],[],tmo)
        v = None
        if r:
            try: v = os.read(fd, 4096).decode().strip()
            except: v = ''
        os.close(fd)
        return v
    except Exception as e:
        return f'ERR {e}'

for i in range(10):
    row = {p: readp(p) for p in ['d0','d1','d2','d3','d4','d5','d6','d7','d9','d11']}
    print(row, flush=True)
    time.sleep(0.5)
