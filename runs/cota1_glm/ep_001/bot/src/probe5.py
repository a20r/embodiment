import os, time, threading
NB=os.O_NONBLOCK
fds={}
def wr_open(p):
    if p not in fds:
        fds[p]=os.open(f'/dev/robot/{p}', os.O_WRONLY|NB)
    return fds[p]
def wr(p,v):
    try: os.write(wr_open(p), f'{v}\n'.encode())
    except Exception as e: return f'ERR:{e}'
def rd(p):
    try:
        fd=os.open(f'/dev/robot/{p}', os.O_RDONLY|NB)
        import select
        out=b''; t0=time.time()
        while time.time()-t0<0.4:
            r,_,_=select.select([fd],[],[],0.1)
            if r:
                d=os.read(fd,8192)
                if d: out=d; break
        os.close(fd)
        return out.decode().strip() or '(empty)'
    except Exception as e: return f'ERR:{e}'
def show(tag):
    print(tag, '| d0',rd('d0'),'| d1',rd('d1'),'| d2',rd('d2'),'| d3',rd('d3'),'| d6',rd('d6'))
    print('    d5',rd('d5'))
    print('    d8',rd('d8'))
# sustained reverse throttle
stop=False
def pump(p,v):
    while not stop: wr(p,v); time.sleep(0.01)
th=threading.Thread(target=pump,args=('d7','-1.0')); th.start()
time.sleep(3); stop=True; th.join()
show('after d7=-1 x3s:')
time.sleep(1); show('1s later:')
# sustained forward
stop=False
th=threading.Thread(target=pump,args=('d7','1.0')); th.start()
time.sleep(3); stop=True; th.join()
show('after d7=+1 x3s:')
time.sleep(1); show('1s later:')
for fd in fds.values(): os.close(fd)
