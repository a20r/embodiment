import time
def rd(p):
    with open('/dev/robot/'+p) as f: return f.readline().strip()
def wr(p,v):
    with open('/dev/robot/'+p,'w') as f: f.write(str(v)+'\n')
def lidar():
    while True:
        try:
            v=[float(x) for x in rd('d3').split(',')]
            if len(v)==16: return v
        except: pass
def heading():
    while True:
        s=rd('d1')
        try: return float(s)
        except: pass
def drive(l,r):
    wr('d10',l); wr('d11',r)
def stop(): drive(0,0)
def status():
    s=rd('d6')
    return s
def goal():
    s=status()
    try: return int(s.split('goal=')[1].split()[0])
    except: return 0
def tx(msg): wr('d0',msg)
