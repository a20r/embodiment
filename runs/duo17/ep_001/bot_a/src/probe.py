import sys, time
sys.path.insert(0,'/bot/src')
import rio as m
def show(tag):
    s=m.snap(); print(tag, 'hd=',s['d4'],'d11=',s['d11'],'d9=',s['d9'],'d6=',s['d6'],'d0=',s['d0'],'d5=',s['d5'],'| lidar4=',s['d2'].split(',')[4] if s['d2'] else None, s['d3'])
port=sys.argv[1]
for cmd in sys.argv[2:]:
    show('  before')
    for i in range(10):
        m.wr(port,cmd,0.2); time.sleep(0.12)
    show('  after %r x10:'%cmd)
    m.wr(port,'0'); time.sleep(0.3)
