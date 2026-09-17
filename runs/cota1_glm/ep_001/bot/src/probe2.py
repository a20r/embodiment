import time
def rd(p):
    try:
        with open(f'/dev/robot/{p}') as f:
            return f.readline().strip()
    except Exception as e:
        return f'ERR:{e}'
t0=time.time()
with open('/dev/robot/d7','w') as f:
    f.write('0.5\n')
for i in range(8):
    time.sleep(0.5)
    print(f'{time.time()-t0:4.1f}s d0={rd("d0")} d1={rd("d1")} d2={rd("d2")} d3={rd("d3")} d5={rd("d5")} d6={rd("d6")}')
    print('   d8:', rd('d8'))
with open('/dev/robot/d7','w') as f:
    f.write('0.0\n')
for i in range(6):
    time.sleep(0.5)
    print(f'stop {time.time()-t0:4.1f}s d3={rd("d3")} d5={rd("d5")}')
