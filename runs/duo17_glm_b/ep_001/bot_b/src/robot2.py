import os, time, math, threading, select

BASE='/dev/robot/'
_PORTS=['d0','d2','d3','d4','d5','d6','d9','d10','d11']
_fds={p: os.open(BASE+p, os.O_RDONLY|os.O_NONBLOCK) for p in _PORTS}
_fd8 = os.open(BASE+'d8', os.O_WRONLY|os.O_NONBLOCK)
_lock=threading.Lock()
_cache={p:'' for p in _PORTS}
_buf={p:'' for p in _PORTS}
_stop=False

def _reader():
    while not _stop:
        for p in _PORTS:
            r,_,_=select.select([_fds[p]],[],[],0)
            if r:
                try:
                    d=os.read(_fds[p],4096).decode(errors='replace')
                    if d:
                        _buf[p]+=d
                        if '\n' in _buf[p]:
                            parts=_buf[p].split('\n')
                            complete=[x for x in parts[:-1] if x.strip()]
                            if complete:
                                with _lock: _cache[p]=complete[-1]
                            _buf[p]=parts[-1]
                except BlockingIOError: pass
        time.sleep(0.05)

_t=threading.Thread(target=_reader,daemon=True); _t.start()

def get(p):
    with _lock: return _cache.get(p,'')

def fget(p, default=0.0):
    v=lastline(get(p))
    try: return float(v)
    except: return default

def heading(): return fget('d4')
def battery(): return fget('d11')
def lastline(v):
    if not v: return ''
    parts=[p for p in v.split('\n') if p.strip()]
    return parts[-1] if parts else ''

def lidar():
    v=lastline(get('d2'))
    if not v: return []
    try: return [float(x) for x in v.split(',')]
    except ValueError: return []

def odom():
    return fget('d9'), fget('d6')

def status():
    return lastline(get('d3'))

def status_d():
    out={}
    for tok in status().split():
        if '=' in tok:
            k,v=tok.split('=',1); out[k]=v
    return out

def motors(l,r):
    with open(BASE+'d1','w') as f: f.write(f'{l}\n')
    with open(BASE+'d7','w') as f: f.write(f'{r}\n')

def stop(): motors(0,0)

def tx(msg):
    try:
        os.write(_fd8,(msg+'\n').encode()); return True
    except Exception: return False

def rx():
    v=get('d10')
    return v if v else None
