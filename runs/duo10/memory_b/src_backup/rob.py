import time
DEV='/dev/robot/'
def rd(p):
    for _ in range(50):
        with open(DEV+p) as f:
            s=f.readline().strip()
        if s: return s
        time.sleep(0.05)
    raise RuntimeError('no data '+p)
def wr(p,v):
    with open(DEV+p,'w') as f: f.write(str(v)+'\n')
def heading(): return float(rd('d1'))
def ranges(): return [float(x) for x in rd('d3').split(',')]
def enc(): return int(rd('d7')), int(rd('d8'))
def motors(l,r): wr('d10',l); wr('d11',r)
def status(): return rd('d6')
def tx(msg): wr('d0',msg)
def rx():
    with open(DEV+'d4') as f:
        import select
        r,_,_=select.select([f],[],[],0.5)
        if r: return f.readline().strip()
    return None
def stop(): motors(0,0)
