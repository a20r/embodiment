import sys, time, json
sys.path.insert(0,'/bot/src')
from rob import *
def lastline(name, timeout=0.3):
    d = read_port(name, timeout).decode(errors='replace')
    lines=[l for l in d.strip().split('\n') if l.strip()]
    return lines[-1] if lines else ''
def drive(l,r,secs):
    write_port('d1',str(l)); write_port('d7',str(r)); time.sleep(secs)
    write_port('d1','0'); write_port('d7','0')
def snap(tag):
    s = {n: lastline(n) for n in ['d0','d3','d4','d5','d6','d9','d11']}
    print(tag, s)
    return s
a=snap('pre ')
A=scan(1.2)
drive(30,30,2.0)
time.sleep(0.5)
b=snap('mid ')
B=scan(1.2)
drive(30,30,2.0)
time.sleep(0.5)
c=snap('post')
C=scan(1.2)
def bb(p):
    xs=[q[0] for q in p]; ys=[q[1] for q in p]; zs=[q[2] for q in p]
    return 'n=%d x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'%(len(p),min(xs),max(xs),min(ys),max(ys),min(zs),max(zs))
print('A:',bb(A)); print('B:',bb(B)); print('C:',bb(C))
json.dump({'A':A,'B':B,'C':C}, open('/memory/drive.json','w'))
