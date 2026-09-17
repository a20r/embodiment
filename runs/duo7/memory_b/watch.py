import time, os
def rd(p):
    with open('/dev/robot/'+p) as f: return f.readline().strip()
seen=0
while True:
    try:
        g=rd('d6')
        if 'goal=1' in g:
            with open('/tmp/ALERT','a') as f: f.write('%.0f MYGOAL %s\n'%(time.time(),g))
    except: pass
    try:
        if os.path.exists('/tmp/radio_rx.log'):
            s=open('/tmp/radio_rx.log').read()
            i=s.find('GOALFOUND',seen)
            # find GOALFOUND lines from alpha only
            for line in s.splitlines()[-50:]:
                if 'GOALFOUND' in line and 'beta' not in line:
                    with open('/tmp/ALERT','a') as f: f.write('%.0f ALPHA %s\n'%(time.time(),line))
                    seen=len(s)
                    break
    except: pass
    time.sleep(2)
