import os, time, select

for p in ['d0','d1','d2','d3','d4','d5','d6','d7','d9','d11']:
    # try read
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_RDONLY | os.O_NONBLOCK)
        r,_,_ = select.select([fd],[],[],0.5)
        if r:
            data = os.read(fd, 4096)
            print(f'{p} READ: {data[:200]!r}')
        else:
            print(f'{p} read: no data (open ok)')
        os.close(fd)
    except Exception as e:
        print(f'{p} read err: {e}')
