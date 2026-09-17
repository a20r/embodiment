import threading, time
ports = ['d1','d2','d3','d4','d5','d6','d7','d8','d9']
latest = {}
def reader(p):
    while True:
        try:
            with open('/dev/robot/'+p) as f:
                for line in f:
                    s=line.strip()
                    latest[p] = (time.time(), s)
                    if p=='d4' and s:
                        with open('/tmp/radio.log','a') as g:
                            g.write(f"{time.time():.1f} {s}\n")
        except Exception as e:
            latest[p] = (time.time(), 'ERR '+str(e))
            time.sleep(0.5)
for p in ports:
    threading.Thread(target=reader, args=(p,), daemon=True).start()
while True:
    time.sleep(0.3)
    t=time.time()
    with open('/tmp/state.txt','w') as f:
        for p in ports:
            v = latest.get(p)
            if v: f.write(f"{p} age={t-v[0]:.1f} {v[1]}\n")
            else: f.write(f"{p} -\n")
