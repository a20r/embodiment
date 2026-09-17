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

tests = [('d1','1'), ('d1','100'), ('d1','F'), ('d7','1'), ('d0','1'), ('d5','1'), ('d6','1'), ('d9','1'), ('d11','1')]
for p,s in tests:
    r = wr(p,s)
    time.sleep(0.5)
    print(f'write {p}={s}: {r} | d3={rd("d3")} d4={rd("d4")} d5={rd("d5")} d6={rd("d6")} d9={rd("d9")} d11={rd("d11")}')
