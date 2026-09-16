import os, time, select

def rd(p, tmo=0.3):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_RDONLY | os.O_NONBLOCK)
        r,_,_ = select.select([fd],[],[],tmo)
        v=None
        if r:
            try: v = os.read(fd,4096).decode().strip()
            except: v=''
        os.close(fd)
        return v
    except Exception as e:
        return f'ERR {e}'

def wr(p, s):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, (s+'\n').encode())
        os.close(fd)
        return 'ok'
    except Exception as e:
        return f'ERR {e}'

print('before:', rd('d4'), rd('d5'), rd('d6'), rd('d9'), rd('d11'))
for p in ['d1','d7']:
    print(p, 'write 0.1 ->', wr(p,'0.1'))
time.sleep(1.0)
print('after d1,d7 writes:', rd('d4'), rd('d5'), rd('d6'), rd('d9'), rd('d11'))
