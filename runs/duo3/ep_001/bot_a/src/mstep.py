import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, compass
from drive import stop
from mouse import walls_here, rd6, DIRS, scan
from mouse3 import Bot3, sensors_hot
def savg(n=4):
    vs=[v for v in (rd6() for _ in range(n)) if v is not None]
    return sum(vs)/len(vs) if vs else 0
def status(bot):
    w,L=walls_here(bot.h)
    print("s=",round(savg(),3),"walls E,N,W,S=",[w[0],w[90],w[180],w[270]],"L=",[round(v,2) for v in L])
def go(bot, seq):
    for D in seq:
        if sensors_hot(): print("HOT!"); return
        bot.face(D)
        l=scan(2)
        if l and 0<l[0]<0.33: print("blocked",D); status(bot); return
        ok=bot.step()
        print("stepped",D,"ok",ok,"s=",round(savg(),3),flush=True)
    status(bot)
bot=Bot3(0,0)
c=compass()
while c is None: c=compass()
bot.align(min(DIRS,key=lambda d: abs(((d-c+540)%360)-180)))
if len(sys.argv)>1 and sys.argv[1]!='st':
    seq=[int(x) for x in sys.argv[1].split(',')]
    go(bot,seq)
else:
    status(bot)
write_port("d8","B manual homing to you; HOLD STILL please")
stop()
