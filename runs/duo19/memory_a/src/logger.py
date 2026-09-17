import rio, time, json, sys
out=open('/bot/src/log.jsonl','a')
while True:
    rec={'t':round(time.time(),2)}
    for p in ['d0','d2','d3','d4','d5','d6','d9','d11']:
        rec[p]=rio.rd(p,0.2)
    m=rio.rd('d10',0.05)
    if m: rec['rx']=m
    out.write(json.dumps(rec)+'\n'); out.flush()
    time.sleep(0.25)
