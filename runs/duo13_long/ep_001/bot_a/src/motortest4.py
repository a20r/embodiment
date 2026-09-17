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

print('stop d1,d7:', wr('d1','0'), wr('d7','0'))
for i in range(4):
    time.sleep(0.6)
    print(f't{i}: d4={rd("d4")} d6={rd("d6")} d9={rd("d9")}')
print('d1=0.3 only:', wr('d1','0.3'))
for i in range(4):
    time.sleep(0.6)
    print(f't{i}: d4={rd("d4")} d6={rd("d6")} d9={rd("d9")}')
print('d1=0:', wr('d1','0'))
time.sleep(0.6)
print('d7=0.3 only:', wr('d7','0.3'))
for i in range(4):
    time.sleep(0.6)
    print(f't{i}: d4={rd("d4")} d6={rd("d6")} d9={rd("d9")}')
print('stop both:', wr('d1','0'), wr('d7','0'))
for i in range(3):
    time.sleep(0.6)
    print(f't{i}: d4={rd("d4")} d6={rd("d6")} d9={rd("d9")}')
