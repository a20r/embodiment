import sys, time
sys.path.insert(0,'/bot/src')
from rob import *
print('d9 before:', read_line('d9'), 'd6:', read_line('d6'))
write_port('d1','-4')
time.sleep(1.2)
write_port('d1','0')
print('after d1=-4: d9:', read_line('d9'), 'd6:', read_line('d6'))
time.sleep(0.5)
print('settle: d9:', read_line('d9'), 'd6:', read_line('d6'))
# spin-drive-spin test for d4
h0 = float(read_line('d4'))
print('h0=',h0)
# spin +90
write_port('d1','6'); write_port('d7','-6')
t0=time.time(); 
while time.time()-t0 < 8.0:
    time.sleep(0.5)
    print(' spin d4=', read_line('d4'), flush=True)
write_port('d1','0'); write_port('d7','0')
time.sleep(0.5)
h1 = float(read_line('d4')); print('h1=',h1)
# drive forward 2s
pts0 = scan(1.0)
write_port('d1','6'); write_port('d7','6')
time.sleep(2.0)
write_port('d1','0'); write_port('d7','0')
time.sleep(0.5)
h2 = float(read_line('d4')); print('after drive h2=',h2, 'd9=',read_line('d9'),'d6=',read_line('d6'))
# spin back
write_port('d1','-6'); write_port('d7','6')
t0=time.time()
while time.time()-t0 < 8.0:
    time.sleep(0.5)
    print(' spinback d4=', read_line('d4'), flush=True)
write_port('d1','0'); write_port('d7','0')
time.sleep(0.5)
h3 = float(read_line('d4')); print('h3=',h3)
print('d11 now:', read_line('d11'), ' d0:', read_line('d0'), ' d5:', read_line('d5'))
