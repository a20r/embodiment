import time
def w(p,v):
    with open(f'/dev/robot/{p}','w') as f: f.write(str(v)+'\n')
def r(p):
    with open(f'/dev/robot/{p}') as f: return f.readline().strip()
def snap(): return dict(h=r('d4'), le=r('d9'), re=r('d6'), bump=r('d5'), f=r('d11'))
s0=snap(); print("base",s0)
w('d1',10); w('d7',10); time.sleep(2); s1=snap(); print("fwd10x2",s1)
w('d1',0); w('d7',0); time.sleep(0.5)
w('d1',100); w('d7',100); time.sleep(2); s2=snap(); print("fwd100x2",s2)
w('d1',0); w('d7',0)
print(r('d2'))
print(r('d3'))
