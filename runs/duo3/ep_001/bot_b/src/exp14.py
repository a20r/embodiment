from ctl import Ctl
import time
b=Ctl(); time.sleep(0.3)
s=b.scan(); print('scan',[round(x,2) for x in s])
i=max(range(16),key=lambda i:s[i])
# rotate so that beam i becomes beam0, using spin wheels directly, closed loop on lidar: crude - use heading
h=b.heading(); target=(h+i*22.5)%360
print('h',h,'target',target)
from ctl import angdiff
t0=time.time()
while time.time()-t0<15:
    h=b.heading(); e=angdiff(target,h)
    if abs(e)<4: break
    w=max(min(e*1.5,60),-60)
    if abs(w)<22: w=22*(1 if w>0 else -1)
    b.wr(4,f'{-w/1.78:.1f}'); b.wr(5,f'{w/1.78:.1f}')
    time.sleep(0.12)
b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
s=b.scan(); print('after scan0',s[0], 'h', b.heading())
for v in ['-6','-10','-15']:
    s0=b.scan()[0]
    b.wr(4,v); b.wr(5,v); time.sleep(1.5); b.wr(4,'0'); b.wr(5,'0'); time.sleep(0.2)
    s1=b.scan()[0]
    print('wheel',v,'dist',round(s0-s1,3),'=> mps',round((s0-s1)/1.5,3))
