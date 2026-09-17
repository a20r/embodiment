import time, json
prev=None
while True:
    try:
        vals={}
        for p in ('d0','d3','d5'):
            with open('/dev/robot/'+p) as f: vals[p]=f.read().strip()[:60]
        key=(vals['d0'],vals['d5'],vals['d3'].split('tx=')[0])
        if key!=prev:
            print(time.strftime('%H:%M:%S'), vals, flush=True)
            open('/memory/watch.log','a').write(time.strftime('%H:%M:%S')+' '+json.dumps(vals)+'\n')
            prev=key
    except Exception as e:
        pass
    time.sleep(2)
