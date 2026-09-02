import time
D='/dev/robot/'
def rd(p, timeout=1.0):
    # read one line from fifo
    try:
        f=open(D+p)
        import select
        r,_,_=select.select([f],[],[],timeout)
        if not r: f.close(); return None
        line=f.readline().strip(); f.close(); return line
    except Exception as e:
        return None
def wr(p, s):
    with open(D+p,'w') as f:
        f.write(str(s)+'\n')
def lidar():
    l=rd('d3')
    if l is None: return None
    try: return [float(x) for x in l.split(',')]
    except: return None
def heading():
    h=rd('d1')
    try: return float(h)
    except: return None
def enc():
    a=rd('d7'); b=rd('d8')
    try: return int(a), int(b)
    except: return None,None
def status():
    return rd('d6')
def drive(l,r):
    wr('d10',l); wr('d11',r)
def rdf(p, tries=5):
    for _ in range(tries):
        v=rd(p)
        if v:
            try: return float(v)
            except: pass
    return None
def enc2():
    return rdf('d7'), rdf('d8')
