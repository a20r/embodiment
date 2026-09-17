import json, time, rio
while True:
    try:
        r=json.loads(open('/bot/src/explore2.log').read().splitlines()[-1])
        msg='A LIVE: odo (%.1f,%.1f) h=%d d11=%s %s. Ack: you park on goal, I home in NW on rising d11; a ROUTE from top of my corridor (-2.8,2.9) helps.'%(r['x'],r['y'],r['h'],rio.rd('d11'),r['d3'][-15:])
        open('/bot/src/tx_msg.txt','w').write(msg+'\n')
    except Exception: pass
    time.sleep(15)
