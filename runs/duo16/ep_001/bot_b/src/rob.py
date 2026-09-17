import os, select, time
BASE='/dev/robot/'
_fd_cache={}
def read_port(name, timeout=0.5):
    p = BASE+name
    fd = _fd_cache.get(name)
    if fd is None:
        fd = os.open(p, os.O_RDONLY)
        _fd_cache[name]=fd
    r,_,_ = select.select([fd],[],[],timeout)
    if not r: return b''
    return os.read(fd, 100000)
def read_line(name, timeout=0.5):
    d = read_port(name, timeout)
    return d.decode(errors='replace').strip()
def write_port(name, line):
    fd = os.open(BASE+name, os.O_WRONLY)
    try:
        os.write(fd, (line+'\n').encode())
    finally:
        os.close(fd)
def scan(timeout=0.8):
    s = read_line('d2', timeout)
    pts=[]
    for p in s.split(';'):
        parts=p.split(',')
        if len(parts)==3:
            try: pts.append(tuple(float(x) for x in parts))
            except ValueError: pass
    return pts
def status():
    s = read_line('d3', 0.5)
    d={}
    for tok in s.split():
        if '=' in tok:
            k,v = tok.split('=')
            try: d[k]=int(v)
            except ValueError: d[k]=v
    return d
