import select, time
def rd(p, t=2.0):
    with open(f'/dev/robot/d{p}') as f:
        r,_,_=select.select([f],[],[],t)
        return f.readline().strip() if r else ''
def wr(p,s):
    with open(f'/dev/robot/d{p}','w') as f: f.write(s+'\n')
def scan(): return [float(x) for x in rd(1).split(',')]
print('h',rd(2)); s0=scan(); print('s0',s0)
wr(5,'45'); time.sleep(0.6)
print('h',rd(2)); s1=scan(); print('s1',s1)
# check shifts
for sh in range(-4,5):
    err=sum(abs(s1[i]-s0[(i+sh)%16]) for i in range(16) if s1[i]>0 and s0[(i+sh)%16]>0)
    print('shift',sh,round(err,2))
