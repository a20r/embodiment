#!/bin/bash
# usage: c.sh "<command>" [wait_seconds]  -- sends command to daemon, waits, prints state
echo "$1" >> /tmp/cmd
sleep ${2:-1}
python3 - << 'PY'
import json
s=json.load(open('/tmp/state.json'))
sc=s['scan'] or []
print("pos=(%.2f,%.2f) h=%.1f enc=(%s,%s) d0=%s d5=%s d11=%.3f goal=%s here=%s busy=%s wheels=%s" % (s['x'],s['y'],s['h'],s['encL'],s['encR'],s['d0'],s['d5'],s['d11'],s['goal'],s['here'],s['busy'],s['wheels']))
print("scan: " + " ".join("--" if v is None else "%.2f"%v for v in sc))
PY
tail -n 3 /tmp/events.log
