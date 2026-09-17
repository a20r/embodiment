import ctl, rio, time, re, sys, json
S=1800.0; DIRS={'N':90,'E':0,'S':270,'W':180}
open('/bot/src/route.pid','w').write(str(__import__('os').getpid()))
LOG=open('/bot/src/route.log','a')
def log(s): LOG.write('%s %s\n'%(time.strftime('%H:%M:%S'),s)); LOG.flush()
def ongoal(): s=rio.rd('d3') or ''; return ('here=1' in s) or ('goal=1' in s)
def leg(d,dist):
    ctl.turn(ctl.angdiff(DIRS[d],ctl.heading(5)), v=15)
    tot=0; res='?'
    while tot<dist*S-100:
        m,res=ctl.forward(min(0.6*S,dist*S-tot), v=50, stopfront=0.27); tot+=m
        if ongoal(): return 'GOAL'
        if res!='dist': break
    return '%s %.2f/%.2f %s'%(d,tot/S,dist,res)
import subprocess, os, signal
done=set(); seen=len(open('/bot/src/rx.log').read().splitlines())
def start_explorer():
    return subprocess.Popen(['python3','/bot/src/explore2.py','60','-0.73','1.1'], cwd='/bot/src', stdout=open('/bot/src/explore2.out','a'), stderr=subprocess.STDOUT)
def stop_explorer(p):
    try: p.kill(); p.wait(timeout=3)
    except Exception: pass
    ctl.stop()
exp=start_explorer(); log('explorer started')
log('route executor waiting')
while True:
    lines=open('/bot/src/rx.log').read().splitlines()
    for ln in lines[seen:]:
        m=re.search(r'ROUTE:\s*([NESW]\s*[\d.]+(?:\s*,\s*[NESW]\s*[\d.]+)*)',ln, re.I)
        if m and m.group(1) not in done:
            stop_explorer(exp); exp=None; done.add(m.group(1)); legs=re.findall(r'([NESW])\s*([\d.]+)',m.group(1),re.I)
            log('executing '+m.group(1)); open('/bot/src/tx_msg.txt','w').write('A: executing ROUTE %s now.\n'%m.group(1))
            results=[]
            for d,dist in legs:
                r=leg(d.upper(),float(dist)); results.append(r); log(r)
                if r=='GOAL': break
            g=ongoal(); sc=ctl.scan(3)
            msg=('A: I AM ON THE GOAL (goal=1). Staying here. Come within 1 min!' if g else 'A: route done, goal=0. legs: %s. front=%.2f L=%.2f R=%.2f. Send corrected ROUTE from my current spot.'%('; '.join(results),sc[0],sc[4],sc[12]))
            log(msg); open('/bot/src/tx_msg.txt','w').write(msg[:238]+'\n')
    seen=len(lines)
    if exp is not None and exp.poll() is not None: log('explorer exited (goal?)'); exp=None
    if ongoal(): open('/bot/src/tx_msg.txt','w').write('A: I AM ON THE GOAL (goal=1), stopped. Come here within 1 min! Signal d11 rises toward me.\n')
    time.sleep(2)
