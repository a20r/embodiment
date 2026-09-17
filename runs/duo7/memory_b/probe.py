import sys, os, time, threading

def reader(path, out):
    try:
        with open(path) as f:
            for line in f:
                out.append((time.time(), path, line.strip()))
    except Exception as e:
        out.append((time.time(), path, 'ERR '+str(e)))

out=[]
for p in ['d5']:
    t=threading.Thread(target=reader,args=('/dev/robot/'+p,out),daemon=True)
    t.start()

def rd(p):
    with open('/dev/robot/'+p) as f:
        return f.readline().strip()

def wr(p,v):
    with open('/dev/robot/'+p,'w') as f:
        f.write(str(v)+'\n')

print('start heading', rd('d1'))
print('lidar', rd('d3'))
# strong forward command
for i in range(10):
    wr('d10', 5)
    wr('d11', 5)
    time.sleep(0.3)
print('heading', rd('d1'))
print('lidar', rd('d3'))
# turn command: left only
for i in range(10):
    wr('d10', 5)
    wr('d11', -5)
    time.sleep(0.3)
print('heading after diff', rd('d1'))
print('lidar', rd('d3'))
time.sleep(1)
for o in out: print(o[1], o[2])
