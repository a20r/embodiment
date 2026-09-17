import os, time, select

def rd(p, tmo=0.3):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd],[],[],tmo)
    v=None
    if r:
        try: v = os.read(fd,4096).decode().strip()
        except: v=''
    os.close(fd)
    return v

def wr(p, s):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, (s+'\n').encode()); os.close(fd); return 'ok'
    except Exception as e:
        return f'ERR {e}'

print('t0:', rd('d3'), rd('d4'), rd('d11'))
print('set d1=0.5 d7=0.5:', wr('d1','0.5'), wr('d7','0.5'))
for i in range(6):
    time.sleep(1)
    print(f't{i+1}:', rd('d3'), rd('d4'), rd('d11'), 'lidar0-2:', rd('d2'))
print('stop:', wr('d1','0'), wr('d7','0'))
