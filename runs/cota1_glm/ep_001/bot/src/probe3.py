import time
def rd(p):
    try:
        with open(f'/dev/robot/{p}') as f:
            return f.readline().strip()
    except Exception as e:
        return f'ERR:{e}'
def wr(p,v):
    try:
        with open(f'/dev/robot/{p}','w') as f:
            f.write(f'{v}\n')
        return 'ok'
    except Exception as e:
        return f'ERR:{e}'
print('write d1=0.5:', wr('d1','0.5'), 'd1 now', rd('d1'))
time.sleep(1)
print('d1',rd('d1'),'d2',rd('d2'),'d3',rd('d3'),'d6',rd('d6'),'d8',rd('d8'))
print('write d6=-0.5:', wr('d6','-0.5'), 'd6 now', rd('d6'))
time.sleep(1)
print('d1',rd('d1'),'d2',rd('d2'),'d3',rd('d3'),'d6',rd('d6'),'d8',rd('d8'))
# continuous write to d7 at 50Hz for 2s
import threading
stop=False
def pump():
    with open('/dev/robot/d7','w') as f:
        while not stop:
            f.write('0.6\n'); f.flush(); time.sleep(0.02)
th=threading.Thread(target=pump); th.start()
t0=time.time()
while time.time()-t0<3:
    time.sleep(0.5)
    print('pump d7:',rd('d3'),rd('d5')[:25],rd('d8'))
stop=True; th.join()
print('d3 after pump:',rd('d3'))
