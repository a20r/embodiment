import time, threading
def w(port, val):
    with open(f'/dev/robot/{port}','w') as f:
        f.write(val+'\n'); f.flush()
def r(port):
    with open(f'/dev/robot/{port}') as f:
        return f.readline().strip()
print("before: d4=",r('d4'),"d2=",r('d2'))
w('d1','1'); w('d7','1')
for i in range(4):
    time.sleep(1)
    print(i, "d4=",r('d4'), "d5=",r('d5'), "d6=",r('d6'), "d11=",r('d11'))
print("d2=",r('d2'))
w('d1','0'); w('d7','0')
