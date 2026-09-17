import time, json, os
try:
    log = json.load(open('/memory/ep6.json'))
except Exception:
    log = []
while True:
    leg = [e for e in log if e['tag'].startswith('leg') and e['tag'].endswith('b')]
    last = leg[-1] if leg else {}
    rx  = [e for e in log if e['tag']=='RX'][-1:]
    line = '%s leg=%s d11=%s best=%s phase-hint=%s' % (
        time.strftime('%H:%M'), last.get('tag','-'), last.get('d11','-'),
        max([e.get('d11',0) for e in log if 'd11' in e] or [0]),
        rx[0]['msg'][:60] if rx else '-')
    open('/memory/ep6sum.log','a').write(line+'\n')
    time.sleep(300)
