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
S={}
S['h']=lastline('d4')
S['pre']=scan(1.5)
S['d9']=lastline('d9'); S['d6']=lastline('d6')
drive(-30,-30,2.0)
time.sleep(0.5)
S['d9b']=lastline('d9'); S['d6b']=lastline('d6')
S['post']=scan(1.5)
S['h2']=lastline('d4')
S['d11']=lastline('d11'); S['d0']=lastline('d0')
def bb(p):
    if not p: return None
    xs=[q[0] for q in p]; ys=[q[1] for q in p]; zs=[q[2] for q in p]
    return 'n=%d x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'%(len(p),min(xs),max(xs),min(ys),max(ys),min(zs),max(zs))
print('pre :', bb(S['pre']))
print('post:', bb(S['post']))
print('odom d9 %s->%s d6 %s->%s'%(S['d9'],S['d9b'],S['d6'],S['d6b']))
print('heading %s -> %s ; d11=%s d0=%s'%(S['h'],S['h2'],S['d11'],S['d0']))
json.dump(S, open('/memory/rev.json','w'))
