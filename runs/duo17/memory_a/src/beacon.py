import sys,time; sys.path.insert(0,'/bot/src'); import rio as m
LOG=open('/bot/src/radio.log','a')
def log(s): LOG.write(time.strftime('%H:%M:%S ')+s+'\n'); LOG.flush()
msg=open('/bot/src/outmsg.txt').read().strip()
last=0; lastmsg=msg
while True:
    try: msg=open('/bot/src/outmsg.txt').read().strip()
    except: pass
    if time.time()-last>3.0:
        d=m.rd('d11'); m.wr('d8', msg.replace('{d11}',str(d)),0.3); last=time.time()
        log('TX d11=%s status=%s'%(d, m.rd('d3')))
    rx=m.rd('d10')
    if rx: log('RX <<< '+rx); open('/bot/src/RX.txt','a').write(rx+'\n')
    time.sleep(0.4)
