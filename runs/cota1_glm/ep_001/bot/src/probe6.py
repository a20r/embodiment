import os, errno
for i in range(9):
    p=f'/dev/robot/d{i}'
    # writable-with-reader?
    try:
        fd=os.open(p, os.O_WRONLY|os.O_NONBLOCK); os.close(fd); w='HAS-READER(writable)'
    except OSError as e: w=f'no:{errno.errorcode[e.errno]}'
    # readable nonblocking?
    try:
        fd=os.open(p, os.O_RDONLY|os.O_NONBLOCK); os.close(fd); r='READABLE'
    except OSError as e: r=f'no:{errno.errorcode[e.errno]}'
    print(f'd{i}: {w:26s} {r}')
