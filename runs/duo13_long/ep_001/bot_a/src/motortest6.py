import os, time, select, threading

stop_flag = False
log = []

def rd(p, tmo=0.3):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd],[],[],tmo)
    v=None
    if r:
        try: v = os.read(fd,4096).decode().strip()
        except: v=''
    os.close(fd)
    return v

def logger():
    while not stop_flag:
        log.append((rd('d3'), rd('d4'), rd('d5'), rd('d6'), rd('d9'), rd('d2')))
        time.sleep(0.1)

t = threading.Thread(target=logger); t.start()

def wr(p, s):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, (s+'\n').encode()); os.close(fd)
    except Exception as e:
        print(f'wr {p} ERR {e}')

time.sleep(1.0)
print('--- d7=30 ---'); wr('d7','30'); time.sleep(2.0)
print('--- d7=0 ---'); wr('d7','0'); time.sleep(1.0)
print('--- d7=-30 ---'); wr('d7','-30'); time.sleep(2.0)
print('--- stop ---'); wr('d7','0'); time.sleep(1.0)
stop_flag = True; t.join()
with open('/bot/src/log6.txt','w') as f:
    for row in log:
        f.write(f'{row[0]} | h={row[1]} d5={row[2]} d6={row[3]} d9={row[4]} | lid={row[5]}\n')
print('logged', len(log), 'rows')
# print compact summary every 5th row
for i in range(0, len(log), 5):
    r = log[i]
    lid = r[5].split(',') if r[5] else ['?']*16
    print(f'{r[0]} h={r[1]} d6={r[3]} d9={r[4]} lid0={lid[0]} lid4={lid[4]} lid8={lid[8]}')
