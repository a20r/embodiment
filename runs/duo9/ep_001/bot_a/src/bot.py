import time

DEV='/dev/robot/'
_w10=open(DEV+'d10','w',buffering=1)
_w11=open(DEV+'d11','w',buffering=1)
_tx=open(DEV+'d0','w',buffering=1)

def speed(l,r):
    _w10.write('%g\n'%l); _w11.write('%g\n'%r)

def tx(msg):
    _tx.write(msg+'\n')

def rd(p):
    for _ in range(50):
        with open(DEV+p) as f:
            s=f.readline().strip()
        if s: return s
    return s

def heading():
    return float(rd('d1'))

def lidar():
    return [float(x) for x in rd('d3').split(',')]

def enc():
    return int(rd('d7')), int(rd('d8'))

def status():
    s=rd('d6')
    d={}
    for kv in s.split():
        k,v=kv.split('=')
        d[k]=int(v)
    return d
