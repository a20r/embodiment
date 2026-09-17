#!/usr/bin/env python3
# Alternate mapper (fresh grid) and wall-follow; handle goal=1 and partner goal announcements.
import subprocess, time, os, json, re, sys
sys.path.insert(0,'/bot/src')
from rio import write
LOG='/tmp/sup.log'
def log(m):
    with open(LOG,'a') as f: f.write(f"{time.time():.1f} {m}\n")
def status():
    try: return json.load(open('/tmp/state.json')).get('d3','')
    except: return ''
def partner_at_goal():
    try: rl=open('/tmp/rx.log').read().splitlines()[-6:]
    except: return False
    return any((re.search(r'goal\s*=\s*1', l, re.I) or 'REACHED THE GOAL' in l.upper()) and not re.search(r'goal\s*=\s*0', l, re.I) for l in rl)
def run(cmd, secs, env=None):
    e=dict(os.environ); e.update(env or {})
    p=subprocess.Popen(cmd, env=e); t0=time.time()
    while p.poll() is None and time.time()-t0<secs+5:
        if 'goal=1' in status(): p.terminate(); return 'goal'
        if partner_at_goal() and env.get('HOME_MODE')!='1': p.terminate(); return 'partner'
        time.sleep(0.5)
    if p.poll() is None: p.terminate()
    return 'done'
i=0
while True:
    i+=1
    if 'goal=1' in status():
        write('d1','0'); write('d7','0')
        open('/tmp/beacon_extra.txt','w').write("*** goal=1 *** I REACHED THE GOAL and am PARKED AT IT. Home in on me: your d11 rises toward me. I stay here.")
        log("AT GOAL - parked")
        try: d11=float(json.load(open('/tmp/state.json')).get('d11','0'))
        except: d11=0
        if d11>0.93 and time.time()-globals().get('_last_reenter',0)>40:
            globals()['_last_reenter']=time.time(); log("partner close: re-entering goal")
            write('d1','-80'); write('d7','-80'); time.sleep(2.0); write('d1','80'); write('d7','80'); time.sleep(2.2); write('d1','0'); write('d7','0')
        time.sleep(5); continue
    if partner_at_goal():
        log("partner reports goal=1 -> homing"); r=run(['python3','/bot/src/mapper.py','600'],600,{'HOME_MODE':'1'}); log(f"home run -> {r}"); continue
    if i%2==1:
        if os.path.exists('/tmp/grid.json'): os.remove('/tmp/grid.json')
        log("mapper 240s"); r=run(['python3','/bot/src/mapper.py','240'],240,{})
    else:
        side='left' if (i//2)%2 else 'right'; log(f"wallfollow {side} 150s"); r=run(['python3','/bot/src/wallfollow.py','150',side],150,{})
    log(f"-> {r}"); write('d1','0'); write('d7','0'); time.sleep(1)
