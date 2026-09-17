import time
def w(p,v):
    with open(f'/dev/robot/{p}','w') as f: f.write(str(v)+'\n')
def r(p):
    with open(f'/dev/robot/{p}') as f: return f.readline().strip()
# back up a bit
w('d1',-20); w('d7',-20); time.sleep(1.5); w('d1',0); w('d7',0)
print("after backup: bump",r('d5'),"f",r('d11'))
print("lidar",r('d2'))
h0=float(r('d4')); l0=int(r('d9')); r0=int(r('d6'))
w('d1',10); w('d7',-10); time.sleep(2); w('d1',0); w('d7',0)
time.sleep(0.3)
h1=float(r('d4')); l1=int(r('d9')); r1=int(r('d6'))
print("spin: dh",h1-h0,"dl",l1-l0,"dr",r1-r0)
print("lidar",r('d2'))
