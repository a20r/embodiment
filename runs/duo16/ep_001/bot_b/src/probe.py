import os, time
base='/dev/robot/'
for name in ['d0','d1','d2','d3','d4','d5','d6','d7','d8','d9','d10','d11']:
    fd = os.open(base+name, os.O_RDONLY | os.O_NONBLOCK)
    try:
        data = os.read(fd, 4000)
    except Exception as e:
        data = b'<err %s>' % str(e).encode()
    os.close(fd)
    print(name, 'READ:', data[:120])
