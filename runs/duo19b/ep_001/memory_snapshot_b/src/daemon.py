import sys; sys.path.insert(0,'/bot/src')
import rio, time, json
# Continuously log radio RX and telemetry
while True:
    m = rio.read_line('d10', 0.5)
    if m:
        line = f"{time.strftime('%H:%M:%S')} RX: {m}"
        with open('/bot/rx.log','a') as f: f.write(line+'\n')
        with open('/memory/rx.log','a') as f: f.write(line+'\n')
    t = {}
    for p in ['d3','d4','d2','d9','d6','d11','d0','d5']:
        t[p] = rio.read_line(p, 0.5)
    with open('/bot/telemetry.log','a') as f:
        f.write(json.dumps({'t': time.time(), **t})+'\n')
    time.sleep(0.3)
