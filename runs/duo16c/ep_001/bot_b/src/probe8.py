import os, time, fcntl

# try opening d0 read-write nonblocking
for p in ['d0','d5','d6','d9','d11','d4']:
    fd = os.open(f'/dev/robot/{p}', os.O_RDWR | os.O_NONBLOCK)
    data = os.read(fd, 200)
    try:
        os.write(fd, b'0\n')
        w = 'write-ok'
    except Exception as e:
        w = f'write-fail: {e}'
    os.close(fd)
    print(p, 'read:', data[:60], w)
