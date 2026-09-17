import time
def rd(p):
    try:
        with open(f'/dev/robot/{p}') as f:
            return f.readline().strip()
    except Exception as e:
        return f'ERR:{e}'
print('before:', rd('d2'), rd('d3'), rd('d5')[:40], rd('d8'))
# steer
with open('/dev/robot/d4','w') as f:
    f.write('0.5\n')
time.sleep(0.5)
print('after steer 0.5:', rd('d2'), rd('d5')[:40])
time.sleep(0.5)
print('after 1s:', rd('d2'), rd('d5')[:40])
with open('/dev/robot/d4','w') as f:
    f.write('0.0\n')
time.sleep(0.5)
print('after steer 0:', rd('d2'), rd('d5')[:40])
