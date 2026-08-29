import time
DEV='/dev/robot/'
def read(p):
    for _ in range(20):
        with open(DEV+p) as f:
            s=f.readline().strip()
        if s: return s
        time.sleep(0.02)
    return s
def write(p,s):
    with open(DEV+p,'w') as f:
        f.write(str(s)+'\n')
def heading(): return float(read('d1'))
def lidar(): return [float(x) for x in read('d3').split(',')]
def enc(): return int(float(read('d7'))), int(float(read('d8')))
def drive(a,b):
    write('d10',a); write('d11',b)
def stop(): drive(0,0)
def status(): return read('d6')
def tx(m): write('d0',m)
def rx(): return read('d4')
