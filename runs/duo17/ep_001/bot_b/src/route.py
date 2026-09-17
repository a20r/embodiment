import ctl,rb,time,json,sys
import leg
from explore import L
ctl.CPU=1920.0
b=ctl.Bot(); last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1]); b.x,b.y=last['x'],last['y']
route=[(90,3.2,('E',2.0)),(0,3.0,('S',1.3)),(270,1.0,('E',0.5)),(0,0.8,('S',0.3)),(270,1.8,('E',1.0)),(0,1.0,None),(270,0.6,None)]
try:
    for i,(H,md,so) in enumerate(route):
        rb.wr('d8',f'B: en route to you, leg {i} heading {H}. my est ({b.x:.1f},{b.y:.1f}). d11={leg.d11(3):.2f}')
        out=leg.leg(b,H,md,so)
        for line in out: L(f'route leg{i} H{H}: {line}')
        if any('HERE=1' in l for l in out): L('!!!!! ARRIVED here=1'); break
        for _ in range(6):
            m=rb.rd('d10',0.3)
            if not m.strip(): break
            open('/bot/src/msgs.txt','a').write(f'{time.time():.0f} RX: {m}\n')
            if 'homing' not in m: L('RX:',m[:150])
    L('route end', rb.rd('d3'))
finally: rb.wr('d1','0'); rb.wr('d7','0')
