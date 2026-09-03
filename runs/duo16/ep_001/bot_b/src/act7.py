import sys, time
sys.path.insert(0,'/bot/src')
from rob import *
def frontstats():
    pts = scan(1.2)
    dead=[p for p in pts if p[0]>0.05 and abs(p[1])<0.15]
    allx=[p[0] for p in pts]
    if dead:
        xs=[p[0] for p in dead]; zs=[p[2] for p in dead]
        ys=[p[1] for p in dead]
        return 'n=%d dead n=%d xmin=%.3f xmax=%.3f y[%.2f,%.2f] z[%.2f,%.2f]'%(len(pts),len(dead),min(xs),max(xs),min(ys),max(ys),min(zs),max(zs))
    return 'n=%d'%len(pts)
print('before:', frontstats(), 'd4=', read_line('d4'), 'd11=', read_line('d11'), 'd0=', read_line('d0'), 'd5=', read_line('d5'), 'd6=', read_line('d6'), 'd9=', read_line('d9'))
write_port('d1','6'); write_port('d7','6')
for i in range(8):
    time.sleep(0.4)
    print(i, frontstats(), 'd11=', read_line('d11'), 'd0=', read_line('d0'), flush=True)
write_port('d1','0'); write_port('d7','0')
time.sleep(1)
print('after:', frontstats(), 'd4=', read_line('d4'), 'd11=', read_line('d11'))
st = status(); print('status:', st)
