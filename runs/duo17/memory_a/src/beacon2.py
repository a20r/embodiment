import sys,time; sys.path.insert(0,'/bot/src'); import rio as m, ctl as c
LOG=open('/bot/src/radio.log','a')
def log(s): LOG.write(time.strftime('%H:%M:%S ')+s+'\n'); LOG.flush()
last=0; lastre=0
def rearrive():
    log('RE-ARRIVAL maneuver start')
    c.turn_to(0,tol=5,spd=35); c.drive(45,45); time.sleep(1.2); c.stop(); time.sleep(2.0)
    log('  out: %s'%m.rd('d3'))
    c.turn_to(180,tol=5,spd=35); t0=time.time(); c.drive(45,45)
    while time.time()-t0<3:
        L=c.lidar()
        if L and 0<L[0]<0.21: break
    c.stop(); time.sleep(0.5); c.turn_to(90,tol=5,spd=35)
    for i in range(6):
        s=m.rd('d3') or ''; log('  back: %s'%s)
        if 'here=1' in s: return True
        c.drive(40,40); time.sleep(0.4); c.stop(); time.sleep(0.3)
    return False
while True:
    try: msg=open('/bot/src/outmsg.txt').read().strip()
    except: msg='A: on goal, here=1'
    if time.time()-last>3.0:
        d=m.rd('d11'); m.wr('d8', msg.replace('{d11}',str(d)),0.3); last=time.time()
        st=m.rd('d3') or ''; log('TX d11=%s status=%s'%(d, st))
        if 'goal=1' in st: log('!!!!! GOAL=1 ACHIEVED'); open('/bot/src/GOAL_DONE','w').write(st)
    rx=m.rd('d10')
    if rx:
        log('RX <<< '+rx); open('/bot/src/RX.txt','a').write(rx+'\n')
        st=m.rd('d3') or ''
        if 'here=1' in rx and 'goal=0' in st and time.time()-lastre>180:
            lastre=time.time(); ok=rearrive(); log('re-arrival done here=%s'%ok)
    time.sleep(0.4)
