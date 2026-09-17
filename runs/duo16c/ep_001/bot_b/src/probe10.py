from scene import *
import time

print('no-cmd consecutive diffs (2s apart):')
for i in range(4):
    a = descriptor(); time.sleep(2); b = descriptor()
    print(f'  {i}: {diff(a,b):.4f}')
print('cmd d1=1 d7=1 diffs:')
for i in range(3):
    a = descriptor(); stream('1.0','1.0',2.0); b = descriptor()
    print(f'  {i}: {diff(a,b):.4f}')
