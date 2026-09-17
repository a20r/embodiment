import rio, time, sys
# usage: drive.py <port> <value> <seconds> [interval]
port, val, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
iv = float(sys.argv[4]) if len(sys.argv)>4 else 1.0
def scan():
    s=rio.rd('d2',0.3)
    return ' '.join('%.2f'%float(v) for v in s.split(',')) if s else '?'
print('start x=%s y=%s h=%s d11=%s d0=%s d5=%s | %s'%(rio.rd('d6'),rio.rd('d9'),rio.rd('d4'),rio.rd('d11'),rio.rd('d0'),rio.rd('d5'),scan()))
print(port,'=',val, rio.wr(port,val))
t0=time.time()
while time.time()-t0<secs:
    time.sleep(iv)
    print('%5.1f x=%s y=%s h=%s d11=%s d0=%s d5=%s %s | %s'%(time.time()-t0,rio.rd('d6'),rio.rd('d9'),rio.rd('d4'),rio.rd('d11'),rio.rd('d0'),rio.rd('d5'),rio.rd('d3'),scan()))
print(port,'= 0', rio.wr(port,'0'))
m=rio.rd('d10',0.3)
if m: print('RX:',m)
