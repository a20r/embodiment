#!/usr/bin/env python3
# Background sensor daemon: sole reader of the FIFOs. Writes latest values to /tmp/state.json (20Hz),
# appends d10 (RX) lines to /tmp/rx.log, and 4Hz samples to /tmp/sensors.log
import os, time, select, json
ports = ["d0","d2","d3","d4","d5","d6","d9","d10","d11"]
fds = {}
for p in ports:
    fds[os.open('/dev/robot/'+p, os.O_RDONLY|os.O_NONBLOCK)] = p
bufs = {fd: b"" for fd in fds}
last = {}; lastfast = 0; lastslow = 0
while True:
    r,_,_ = select.select(list(fds),[],[],0.05)
    now = time.time(); idle=False
    for fd in r:
        try: chunk = os.read(fd, 65536)
        except BlockingIOError: continue
        if not chunk:
            idle=True; continue
        bufs[fd] += chunk
        while b"\n" in bufs[fd]:
            line, bufs[fd] = bufs[fd].split(b"\n",1)
            p = fds[fd]; s = line.decode(errors='replace')
            last[p] = s
            if p == "d10":
                with open('/tmp/rx.log','a') as f: f.write(f"{now:.2f} {s}\n")
    if idle: time.sleep(0.01)
    if now - lastfast > 0.04:
        lastfast = now; last['t'] = f"{now:.2f}"
        with open('/tmp/state.json.tmp','w') as f: json.dump(last, f)
        os.replace('/tmp/state.json.tmp', '/tmp/state.json')
    if now - lastslow > 0.25:
        lastslow = now
        with open('/tmp/sensors.log','a') as f: f.write(json.dumps(last)+"\n")
