import os, time, threading
def rd(p):
    try:
        fd=os.open(f'/dev/robot/{p}', os.O_RDONLY|os.O_NONBLOCK)
        import select
        r=b''
        t0=time.time()
        while time.time()-t0<0.5:
            rr,w,_=select.select([fd],[],[],0.1)
            if rr:
                r=os.read(fd,4096)
                if r: break
        os.close(fd)
        return r.decode().strip() or '(empty)'
    except Exception as e:
        return f'ERR:{e}'
def wr(p,v,hold=0.0):
    try:
        fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd, f'{v}\n'.encode())
        if hold: time.sleep(hold)
        os.close(fd)
        return 'ok'
    except Exception as e:
        return f'ERR:{e}'
print('d7 state before:', rd('d3'), rd('d5')[:30])
print('wr d7=1.0:', wr('d7','1.0'))
time.sleep(1.0)
print('after 1s:', rd('d3'), rd('d5')[:30], rd('d8'))
print('wr d7=100:', wr('d7','100'))
time.sleep(1.0)
print('after 1s:', rd('d3'), rd('d5')[:30], rd('d8'))
print('wr d7=-1:', wr('d7','-1'))
time.sleep(1.0)
print('after 1s:', rd('d3'), rd('d5')[:30], rd('d8'))
print('wr d4=1.0:', wr('d4','1.0'))
time.sleep(1.0)
print('after 1s:', rd('d2'), rd('d5'), rd('d8'))
print('wr d4=-1.0:', wr('d4','-1.0'))
time.sleep(1.0)
print('after 1s:', rd('d2'), rd('d5'), rd('d8'))
print('wr d4=0 d7=0')
wr('d4','0'); wr('d7','0')
