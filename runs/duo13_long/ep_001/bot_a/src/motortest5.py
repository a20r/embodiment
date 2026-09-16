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
        log.append((time.time(), rd('d3'), rd('d4'), rd('d5'), rd('d6'), rd('d9'), rd('d11'), rd('d2')))
        time.sleep(0.1)

t = threading.Thread(target=logger); t.start()

def wr(p, s):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, (s+'\n').encode()); os.close(fd)
    except Exception as e:
        print(f'wr {p} ERR {e}')

time.sleep(1.5)
print('--- d1=30 ---'); wr('d1','30'); time.sleep(2.5)
print('--- d1=0 ---'); wr('d1','0'); time.sleep(1.5)
print('--- d7=0.5 ---'); wr('d7','0.5'); time.sleep(2.5)
print('--- d7=1.0 ---'); wr('d7','1.0'); time.sleep(2.0)
print('--- d7=0 ---'); wr('d7','0'); time.sleep(1.5)
print('--- d1=-30 ---'); wr('d1','-30'); time.sleep(2.0)
print('--- stop ---'); wr('d1','0'); time.sleep(1.0)
stop_flag = True; t.join()
for row in log:
    print(row[1], row[2], 'd5=%s d6=%s d9=%s d11=%s' % row[3:7], 'lidar[4]=%s lidar[0]=%s' % (row[7].split(',')[4] if row[7] else '?', row[7].split(',')[0] if row[7] else '?'))
