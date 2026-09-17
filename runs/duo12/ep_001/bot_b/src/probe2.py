import time
def w(port, val):
    with open(f'/dev/robot/{port}','w') as f:
        f.write(str(val)+'\n')
def r(port):
    with open(f'/dev/robot/{port}') as f:
        return f.readline().strip()
def snap():
    return dict(d4=r('d4'), d5=r('d5'), d6=r('d6'), d9=r('d9'), d0=r('d0'), d11=r('d11'))
print("base",snap())
print("== d1=10 alone ==")
w('d1',10); time.sleep(2); print(snap()); w('d1',0)
time.sleep(1)
print("== d7=10 alone ==")
w('d7',10); time.sleep(2); print(snap()); w('d7',0)
time.sleep(1)
print("end",snap())
print(r('d2'))
