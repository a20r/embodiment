import sys, time
sys.path.insert(0,'/bot/src')
from rob import *
pts = scan(1.5)
with open('/memory/scan0.txt','w') as f:
    for p in pts: f.write('%f,%f,%f\n'%p)
print('captured', len(pts))
